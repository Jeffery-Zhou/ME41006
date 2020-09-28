import socket
import time
import cv2
import numpy as np


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print("start camera...")
time.sleep(1)

#connection set up
HOST = '192.168.1.6' #**check host ip
PORT = 8001
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen(5)

#parameters for pose estimation
mtx = np.array([[862.95693399, 0., 355.06015428],
                [0., 859.4439399, 228.77007565],
                [0., 0., 1.]])
dist = np.array([5.59745642e-02, -1.10551114e+01, -4.63555574e-05, -3.08611069e-03,
                 4.56662196e+01])


last_sx=float('inf')
last_sy=float('inf')

#get position of 'object', 'obstacles', and 'goal'
def get_position():
    global last_sx
    global last_sy
    get_sx = float('inf')
    get_sy = float('inf')
    get_gx = float('inf')
    get_gy = float('inf')
    get_ox = []
    get_oy = []
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    if markerIds is not None:
        # ret=(array(rvec),array(tvec),array(_))
        ret = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.066, mtx, dist)
        # ---rvec:rotation vector, attitude of marker respect to camera frame
        # ---rvec=[[[rvec_1, rvec_2,...]]]
        # ---tvec:translation vector, position of marker in camera frame
        # ---tvec=[[[tvec_1, tvec_2...]]]
        for i in range(len(markerIds)):
            rvec = ret[0][i][0]
            tvec = ret[1][i][0]
            #object position
            if markerIds[i] == 33:
                get_sx = tvec[1] * 100 #*100:change unit form m to cm
                get_sy = tvec[0] * 100
                #print("sx:", get_sx, "sy:", get_sy)
            #goal position
            elif markerIds[i] == 20:
                get_gx = tvec[1] * 100
                get_gy = tvec[0] * 100
                #print("gx:", get_gx, " gy:", get_gy)
            else:
                #obstacles position
                x = tvec[1] * 100
                y = tvec[0] * 100
                get_ox.append(x)
                get_oy.append(y)

            cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.03)

        cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds, (0, 0, 255))
        if get_sx != float('inf') and get_sy != float('inf') :
            last_sx = get_sx
            last_sy = get_sy
        else:
            get_sx=last_sx
            get_sy=last_sy
            print("No object and goal !!!!")
    else:
        cv2.putText(frame, "No Ids", (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    return get_sx, get_sy, get_gx, get_gy, get_ox, get_oy

ok=True
#make sure goal and object are detected
while ok:
    sx, sy, gx, gy, ox, oy = get_position()
    if sx != float('inf') and sy != float('inf') and gx != float('inf') and gy != float('inf'):
        ok=False

set=True

while True:
    sx, sy, gx, gy, ox, oy=get_position()
    #detecting start points
    if set:
        ori_gx=gx
        ori_gy=gy
        ori_ox=ox
        ori_oy=oy
        set=False
    a=[int(sx), int(sy), int(ori_gx), int(ori_gy)]
    #add obstacles
    for z in range(len(ori_ox)):
        a.append(int(ori_ox[z]))
        a.append(int(ori_oy[z]))
    a.append('s')
    print("position_array=",a)
    #send message
    for i in range(len(a)):
        connection, address = sock.accept()
        try:
            connection.settimeout(10)
            buf = connection.recv(1024)
            if buf:
                msg = str(a[i])
                print("msg=",msg)
                connection.send(bytearray(msg.encode()))
                print('Sending success!')
            else:
                connection.send(b'please go out!')
        except socket.timeout:
            print('time out')
        connection.close()
