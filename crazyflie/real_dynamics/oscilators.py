from crazyflie_py import Crazyswarm
import numpy as np


Z = 1.0
TAKEOFF_DURATION = 2.5
T = 20 #set time that the simulation last
RATE = 50
T_LAP = T / 2 #time to take one lap
A = 0.4
VX = 0.15

def main():

    #Set instances
    swarm = Crazyswarm()#Create drone
    timehelper = swarm.timeHelper

    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION) #drone takes off
    timehelper.sleep(TAKEOFF_DURATION + 0.5)

    start = np.array(cf.initialPosition) + [0,0,Z] # sets up the initial position
    w = 2 * np.pi /T_LAP # compute the angular velocity 

    t0 = timehelper.time() #get current time to start the count

    while timehelper.time() - t0 < T:
        t = timehelper.time() - t0

        pos = start + np.array([VX*t, A*np.sin(w*t), 0.0])
        v = np.array([VX, A*w*np.cos(w*t), 0.0 ])
        a = np.array([0.0, -A*w**2*np.sin(w*t), 0.0])

        yaw = 0.0
        omega = np.array([0.0, 0.0, 0.0])

        cf.cmdFullState(pos, v, a, yaw, omega)
        timehelper.sleepForRate(RATE)

    cf.notifySetpointsStop() #change to high level
    cf.land(targetHeight=0.03, duration=2.0)
    timehelper.sleep(2.5)

if __name__ == '__main__': #call main when file is executed
    main()