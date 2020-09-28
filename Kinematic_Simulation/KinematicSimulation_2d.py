import numpy as np
import matplotlib.pyplot as plt
import math

l_1=0.5 #length of link 1 [m]
l_2=0.5 #lenth of link 2 [m]

L=[l_1,l_2] #link parameters
th=[0.0*math.pi,0.0*math.pi] #initial angles
P=[0.5,0.1]


def InverseKinematics(X,L):

    l1,l2=L
    x,y=X
    l = math.hypot(X[0], X[1])

    #angle of the link 1
    if ((math.pow(l1,2)+math.pow(l,2)-math.pow(l2,2))/(2*l1*l))>1:
        th1=math.atan2(y,x)
        print('th1 error {},{}'.format(x,y))
    elif ((math.pow(l1,2)+math.pow(l,2)-math.pow(l2,2))/(2*l1*l))<-1:
        th1=math.atan2(y,x)-math.pi
        print('th1 error {},{}'.format(x, y))
    else:
        th1=math.atan2(y,x)-math.acos((math.pow(l1,2)+math.pow(l,2)-math.pow(l2,2))/(2*l1*l))
    #angle of link 2
    if((math.pow(l1,2)+math.pow(l2,2)-math.pow(l,2))/(2*l1*l2))>1:
        th2=math.pi
        print('th2 error {},{}'.format(x,y))
    elif ((math.pow(l1,2)+math.pow(l2,2)-math.pow(l,2))/(2*l1*l2))<-1:
        th2=math.pi
        print('th2 error {},{}'.format(x, y))
    else:
        th2=math.pi-math.acos((math.pow(l1,2)+math.pow(l2,2)-math.pow(l,2))/(2*l1*l2))

    return [th1,th2]

def ForwardKinematics(th,L):
    l1,l2=L
    th1,th2=th

    x0=0.0
    y0=0.0

    #position of tip of link1
    x1= l1*math.cos(th1)
    y1= l1*math.sin(th1)
    #position of tip of link2
    x2=x1+l2*math.cos(th1+th2)
    y2=y1+l2*math.sin(th1+th2)
    #position of Gripper
    #Rotation matrix
    R=np.array([[math.cos(th1+th2),-1*math.sin(th1+th2),x2],[math.sin(th1+th2),math.cos(th1+th2),y2]])
    T1=np.array([[0],[-0.1],[1]])
    T2 = np.array([[0], [0.1], [1]])
    T3 = np.array([[0.1], [-0.1], [1]])
    T4 =np.array([[0.1], [0.1], [1]])
    T5=np.array([[0.1],[0],[1]])
    G1=np.dot(R,T1)
    G2=np.dot(R,T2)
    G3 = np.dot(R, T3)
    G4 = np.dot(R, T4)
    G5 = np.dot(R, T5)
    X=np.array([[x0,x1],[y0,y1],[x1,x2],[y1,y2],[x2,G1[0][0]],[y2,G1[1][0]],[x2,G2[0][0]],[y2,G2[1][0]],[x2,G5[0][0]],[y2,G5[1][0]],[G1[0][0],G3[0][0]],[G1[1][0],G3[1][0]],[G2[0][0],G4[0][0]],[G2[1][0],G4[1][0]]])

    return X


def update(X,orith):
    move=[0.0*math.pi,0.0*math.pi]
    th=InverseKinematics(X,L)
    move[0]=orith[0]-th[0]
    move[1]=orith[1]-th[1]
    move_th=[orith[0],orith[1]]
    i=0
    while i<100:
        move_th[0]=move_th[0]-move[0]/100
        move_th[1]=move_th[1]-move[1]/100
        # calculation of forward kinematics
        p=ForwardKinematics(move_th,L)
        print("p=",p)
        ax.cla()
        plt.title('InverseKinematics_2links_2DOF')
        #plt.axis('equal')
        ax.set_xlim([-1.2, 1.2])
        ax.set_ylim([-1.2, 1.2])
        plt.grid(True)
        #update the position of each point
        ax.plot(p[2], p[3], linewidth=5, color="blue")
        ax.plot(p[0], p[1], 'o-', markersize=15,
                markerfacecolor="green", linewidth=5, color="blue")
        ax.plot(p[12], p[13], linewidth=5, color="orange")
        ax.plot(p[10], p[11], linewidth=5, color="orange")
        ax.plot(p[8], p[9], linewidth=5, color="orange")
        ax.plot(p[6], p[7], linewidth=5, color="orange")
        ax.plot(p[4], p[5], linewidth=5, color="orange")

        plt.pause(0.01)
        #re-draw of the links

        i=i+1

    return th


last_th=InverseKinematics(P,L)
p= ForwardKinematics(last_th,L)
fig,ax=plt.subplots()
plt.title('InverseKinematics_2links_2DOF')
#plt.axis('equal')
ax.set_xlim([-1.2,1.2])
ax.set_ylim([-1.2,1.2])

#plot the graph
plt.grid(True)
ax.plot(p[2],p[3],linewidth = 5, color="blue")
ax.plot(p[0],p[1], 'o-', markersize=15,
                     markerfacecolor="green", linewidth = 5, color="blue")
ax.plot(p[12], p[13], linewidth=5, color="orange")
ax.plot(p[10], p[11], linewidth=5, color="orange")
ax.plot(p[8], p[9], linewidth=5, color="orange")
ax.plot(p[6], p[7], linewidth=5, color="orange")
ax.plot(p[4],p[5],linewidth = 5, color="orange")


plt.pause(0.1)

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
    X=[np.float(x),np.float(y)]
    if np.hypot(X[0],X[1])<=1:
        last_th=update(X,last_th)

    else:
        print("Error! Out of range.")
print('[Info] Delete Figure window to exit program')
plt.show()
