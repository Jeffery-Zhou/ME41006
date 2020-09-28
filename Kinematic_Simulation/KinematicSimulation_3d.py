import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import math
import time



l_1=1#length of link 1 [m]
l_2=1 #lenth of link 2 [m]

L=[l_1,l_2] #link parameters
th=[0.0*math.pi,0.0*math.pi] #initial angles
P=[2,0,0]


def InverseKinematics(X,L):
    l1,l2=L
    x,y,z=X
    l = math.hypot(math.hypot(x,y),z)
    r=math.hypot(x,y)
    #angle of theta yaw
    thyaw=math.atan2(y,x)
    #angle of theta1
    if ((math.pow(l1,2)+math.pow(l,2)-math.pow(l2,2))/(2*l1*l))>1:
        th1=math.atan2(y,x)
        print('th1 error {},{}'.format(x,y))
    elif ((math.pow(l1,2)+math.pow(l,2)-math.pow(l2,2))/(2*l1*l))<-1:
        th1=math.atan2(y,x)-math.pi
        print('th1 error {},{}'.format(x, y))
    else:
        th1=math.atan2(z,r)-math.acos((math.pow(l1,2)+math.pow(l,2)-math.pow(l2,2))/(2*l1*l))
    
    # angle of theta2
    if((math.pow(l1,2)+math.pow(l2,2)-math.pow(l,2))/(2*l1*l2))>1:
        th2=math.pi
        print('th2 error {},{}'.format(x,y))
    elif ((math.pow(l1,2)+math.pow(l2,2)-math.pow(l,2))/(2*l1*l2))<-1:
        th2=math.pi
        print('th2 error {},{}'.format(x, y))
    else:
        th2=math.pi-math.acos((math.pow(l1,2)+math.pow(l2,2)-math.pow(l,2))/(2*l1*l2))
    

    return [thyaw,th1,th2]

def ForwardKinematics(th,L):
    l1,l2=L
    thyaw,th1,th2=th

    x0=0.0
    y0=0.0
    z0=0.0

    #position of tip of link1
    x1= l1*math.cos(th1)*math.cos(thyaw)
    y1= l1*math.cos(th1)*math.sin(thyaw)
    z1= l1*math.sin(th1)
    #position of tip of link2
    x2=x1+l2*math.cos(th1+th2)*math.cos(thyaw)
    y2=y1+l2*math.cos(th1+th2)*math.sin(thyaw)
    z2=z1+l2*math.sin(th1+th2)
    l = math.hypot(math.hypot(x2-x1, y2-y1), z2-z1)

    X=np.array([[[x0,y0,z0],[x1,y1,z1]],[[x1,y1,z1],[x2,y2,z2]]])

    return X


def update(X,orith):
    move=[0.0*math.pi,0.0*math.pi,0.0*math.pi]
    th=InverseKinematics(X,L)
    move[0] = orith[0] - th[0]
    move[1] = orith[1] - th[1]
    move[2] = orith[2] - th[2]
    move_th = [orith[0], orith[1],orith[2]]
    i=0
    while i<100:

        move_th[0]=move_th[0]-move[0]/100
        move_th[1]=move_th[1]-move[1]/100
        move_th[2]=move_th[2]-move[2]/100
        # calculation of forward kinematics
        p=ForwardKinematics(move_th,L)
        print("p=",p)
        ax.cla()
        ax.set_xlim3d(-2, 2)
        ax.set_ylim3d(-2, 2)
        ax.set_zlim3d(-2, 2)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_axisbelow(True)  # send grid lines to the background
        #update the position of each point
        ax.plot(p[1].T[0], p[1].T[1], p[1].T[2], 'o-', markersize=10,
                markerfacecolor="red", linewidth=5, color="blue")

        ax.plot(p[0].T[0], p[0].T[1], p[0].T[2], 'o-', markersize=10,
                markerfacecolor="green", linewidth=5, color="blue")

        plt.pause(0.01)
        i=i+1
    return p,th






fig = plt.figure('InverseKinematics_2links_3DOF')  #create the framedof
ax = plt.axes([0.05, 0.2, 0.90, .75], projection='3d') #3d ax panel
plt.title('InverseKinematics_2links_3dof')
ax.set_xlim3d(-2, 2)
ax.set_ylim3d(-2, 2)
ax.set_zlim3d(-2, 2)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_axisbelow(True) #send grid lines to the background
last_th=InverseKinematics(P,L)
p= ForwardKinematics(last_th,L)
#plot the graph

ax.plot(p[1].T[0], p[1].T[1], p[1].T[2], 'o-', markersize=10,
                markerfacecolor="red", linewidth=5, color="blue")

ax.plot(p[0].T[0], p[0].T[1], p[0].T[2], 'o-', markersize=10,
                markerfacecolor="green", linewidth=5, color="blue")
plt.pause(2)



while True:

    print("input x (input 'q' to stop):")
    x=raw_input()
    if x=='q':
        print("End!!")
        break
    print("input y (input 'q' to stop):")
    y=raw_input()
    if y=='q':
        print("End!!")
        break
    print("input z:")
    z=raw_input()
    if z=='q':
        print("End!!")
        break
    X=[np.float(x),np.float(y),np.float(z)]
    if np.hypot(np.hypot(X[0], X[1]), X[2]) <= math.sqrt(3):
        X,last_th=update(X, last_th)
    else:
        print("Error! Out of range.")

print('[Info] Delete Figure window to exit program')
plt.show()


