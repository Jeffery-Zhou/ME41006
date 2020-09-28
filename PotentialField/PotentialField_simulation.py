import numpy as np
import matplotlib.pyplot as plt

show_animation = True
dgoal = 5#distance from goal
Katt = 10
Krep=100
rate = 0.01
od=5 #dsitance from obstacle


def attractive_field(x_currt,y_currt,gx,gy):
    d = np.hypot(x_currt - gx, y_currt - gy)#np.hypot:Equivalent to sqrt(x1**2 + x2**2)
    if d <= dgoal:
        x = Katt * (x_currt - gx)
        y = Katt * (y_currt - gy)
    elif d > dgoal:
        x = dgoal * (Katt * (x_currt - gx)) / d
        y = dgoal * (Katt * (y_currt - gy)) / d
    return x,y


def repulsive_field(x_currt, y_currt, ox, oy):
    # if no obstacle
    if not ox or not oy:
        return 0, 0
    else:
        mind = float('inf')
        #find the nearest obstacle
        for i, _ in enumerate(ox):
            d = np.hypot(x_currt - ox[i], y_currt - oy[i]) #np.hypot:Equivalent to sqrt(x1**2 + x2**2)
            if mind > d:
                mind = d
                minid = i
        if mind <= od:
            x = Krep * (1 / od - 1 / mind) * (1 / (mind ** 2)) * (x_currt - ox[minid])
            y = Krep * (1 / od - 1 / mind) * (1 / (mind ** 2)) * (y_currt - oy[minid])
        else:
            x = 0
            y = 0
        return x, y


def potential_field(sx,sy,gx,gy,ox,oy):

    x_currt = sx
    y_currt = sy
    d = np.hypot(sx - gx, sy - gy)
    #reach goal if distance between object and goal less than 0.5
    while d>0.5:
        xatt, yatt = attractive_field(x_currt, y_currt, gx, gy)
        xrep, yrep = repulsive_field(x_currt, y_currt, ox, oy)
        minx=xatt+xrep
        miny=yatt+yrep
        x_currt = x_currt - rate * minx
        y_currt = y_currt - rate * miny
        d = np.hypot(x_currt - gx, y_currt - gy)
        print("x_currt=",x_currt)
        print("y_currt=", y_currt)
        print('d=', d)
        #draw
        plt.plot(x_currt, y_currt, ".r")
        plt.pause(0.01)
    print("Goal!!!")
    return d


def main():
    print("potential_field_planning start")
    print("run")
    sx = 0.0 #start x position
    sy = 0.0 #start y position
    gx = 50.0 #goal x position
    gy = 50.0 #goal y position
    ox = [23,25]  # obstacle x position list
    oy = [22,28]  # obstacle y position list

    if show_animation:
        plt.grid(True)
        plt.axis("equal")
    #draw points
    plt.plot(sx, sy, "*k")
    plt.plot(gx, gy, "*m")
    plt.plot(ox,oy,"*b")

    _=potential_field(sx, sy, gx, gy, ox, oy)

    if show_animation:
        print("[Info] Delete Figure window to exit program")
        plt.show()



if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
