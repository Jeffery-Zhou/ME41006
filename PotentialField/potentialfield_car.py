#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty, time
import numpy as np
import matplotlib.pyplot as plt
import cv2
import socket
import time
#**check host ip
HOST = '192.168.1.4'
PORT = 8001
#set parameters
a=[]
show_animation = True
dgoal = 20
Katt = 5
Krep=30000
rate = 0.3
original_sx = 0
original_sy = 0
original_gx=0
original_gy=0
original_ox=[]
original_oy=[]
od=30
flag=True
reach=False



mtx = np.array([[862.95693399, 0., 355.06015428],
                [0., 859.4439399, 228.77007565],
                [0., 0., 1.]])
dist = np.array([5.59745642e-02, -1.10551114e+01, -4.63555574e-05, -3.08611069e-03,
                 4.56662196e+01])

#ros setting
settings = termios.tcgetattr(sys.stdin)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('teleop_twist_keyboard')
speed = rospy.get_param("~speed", 0.5)
turn = rospy.get_param("~turn", 1.0)
move_x = 0
move_th = 0


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


#x,y
def move(move_x,move_y):
    try:
        #print(vels(speed, turn))
        print("x=", move_x)
        print("y=", move_y)
        twist = Twist()
        twist.linear.x = move_y # go forward or bock
        twist.linear.y = move_x # parallel move right or left
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0 * turn
        #print("twist", twist)
        pub.publish(twist)
        time.sleep(0.5)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def attractive_field(x_currt,y_currt,gx,gy):

    x_currt=np.float(x_currt)
    y_currt=np.float(y_currt)
    gx=np.float(gx)
    gy=np.float(gy)
    d = np.hypot(x_currt - gx, y_currt - gy)
    if d <= dgoal:
        x = Katt * (x_currt - gx)
        y = Katt* (y_currt - gy)

    else:
        x =dgoal * (Katt * (x_currt - gx)) / d
        y = dgoal * (Katt * (y_currt - gy)) / d
    print("xatt=",x,"yatt=",y)
    return x,y

def repulsive_field(x_currt,y_currt,ox,oy):
    if not ox or not oy:
        print("No obstacles !!!")
        return 0,0
    else:
        mind = float('inf')
        for i, _ in enumerate(ox):
            d = np.hypot(x_currt - ox[i], y_currt - oy[i])
	    print("obstacle d:",d)
            if mind > d:
                mind = d
                minid = i
        if mind <= od:
            x = Krep * (1 / od - 1 / mind) * (1 / (mind ** 2)) * (x_currt - ox[minid])
            y = Krep * (1 / od - 1 / mind) * (1 / (mind ** 2)) * (y_currt - oy[minid])
        else:
            x = 0
            y = 0
        print("xrep=", x, "yrep=", y)
        return x, y

def set_orintation(sx,sy,gx,gy,ox,oy):
    global original_sx
    global original_sy
    global original_gx
    global original_gy
    global original_ox
    global original_oy
    original_sx = sx
    original_sy = sy
    original_gx = gx - original_sx
    original_gy = gy - original_sy
    if original_gx < 0:
        original_gx = -1.0 * original_gx
    if original_gy < 0:
        original_gy = -1.0 * original_gy

    for i, _ in enumerate(ox):
        ori_ox = ox[i] - original_sx
        ori_oy = oy[i] - original_sy
        if ori_ox < 0:
            ori_ox = -1.0 * ori_ox
        if ori_oy < 0:
            ori_oy = -1.0 * ori_oy
        original_ox.append(ori_ox)
        original_oy.append(ori_oy)
    #print(original_sx, original_sy, original_gx, original_gy, original_ox, original_oy)

def get_orintation():
    global original_sx
    global original_sy
    global original_gx
    global original_gy
    global original_ox
    global original_oy
    print("original:",original_sx, original_sy, original_gx, original_gy, original_ox, original_oy)
    return original_sx,original_sy,original_gx,original_gy,original_ox,original_oy

def calculation(sx, sy, gx, gy, ox, oy):
    osx, osy, ogx, ogy, oox, ooy = get_orintation()
    osx = sx - osx
    osy = sy - osy
    if osx < 0:
        osx = -1.0 * osx
    if osy < 0:
        osy = -1.0 * osy

    #print("orientation:",osx, osy, ogx, ogy, oox, ooy)
    return osx, osy, ogx, ogy, oox, ooy

def get_position():
    get_a=[]
    get_ox=[]
    get_oy=[]
    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, PORT))
        time.sleep(0.01)
        sock.send(b'1')
        msg = sock.recv(1024).decode()
        sock.close()
        if msg == 's':
            #print("position_array=", get_a)
            break
        else:
            get_a.append(int(msg))
    
    for i in range(len(get_a) - 4):
        if ((5 + i) % 2 != 0):
            get_ox.append(get_a[4+i])
        else:
            get_oy.append(get_a[4+i])

    return get_a[0],get_a[1],get_a[2],get_a[3],get_ox,get_oy





def potential_field(sx,sy,gx,gy,ox,oy):
    	
	sx, sy, gx, gy, ox, oy = calculation(sx, sy, gx, gy, ox, oy)
	d = np.hypot(sx - gx, sy - gy)
	print('d=', d)
	if d<=1:
		return d
	print("x_currt=", sx)
	print("y_currt=", sy)
	print("gx=",gx)
	print("gy=",gy)
	xatt, yatt = attractive_field(sx, sy, gx, gy)
	xrep, yrep = repulsive_field(sx, sy, ox, oy)
	minx=xatt+xrep
	miny=yatt+yrep
	print("minx=",minx)
	print("miny=",miny)
	
	move(-rate*0.01*minx,-rate*0.01*miny)
	sx = sx - rate* minx
	sy = sy - rate* miny
	
	print("sx=",sx)
	print("sy=",sy)
	plt.plot(sx, sy, ".r")
	plt.pause(0.01)
	return d


print("potential_field_planning start")

if show_animation:
    plt.grid(True)
    plt.axis("equal")


d= float('inf')
_=getKey()
y=ord('r')
set=True
run=False
while True:
    sx, sy, gx, gy, ox, oy = get_position()
    print("get_msg:",sx, sy, gx, gy, ox, oy )
    # set
    if set:
        print("start set !!")
        set_orintation(sx, sy, gx, gy, ox, oy)
        sx, sy, gx, gy, ox, oy=get_orintation()
        plt.plot(0, 0, "*k")
        plt.plot(gx, gy, "*m")
        plt.plot(ox, oy, "*b")
        time.sleep(5)
	run=True
	print("start run !!")
	set=False
    # run
    if run:
        d = potential_field(sx, sy, gx, gy, ox, oy)
        if d <= 2.5:
	    print("Reach Goal !!!")
            break



if show_animation:
    plt.show()

cap.release()
cv2.destroyAllWindows()
