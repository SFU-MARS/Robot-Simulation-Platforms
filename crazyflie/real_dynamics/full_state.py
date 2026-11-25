from crazyflie_py import Crazyswarm
import numpy as np

Z = 1.0
TAKEOFF_DURATION = 2.5
T = 10 #set time that the simulation last
RATE = 50
VEL = np.array([0.2,0,0]) #set up initial velocity for the drone in m/s
BACK_VEL = np.array([-0.2, 0,0])
DT = 1.0/RATE # time to wait between each step

def main():

    #Set instances
    swarm = Crazyswarm()#Create drone
    timehelper = swarm.timeHelper

    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION) #drone takes off
    timehelper.sleep(TAKEOFF_DURATION)

    start = np.array(cf.initialPosition) + [0,0,Z] # sets up the initial position

    t0 = timehelper.time() #get current time to start the count
    pos = start.copy() #get a copy of the current position

    #Loop until current time - start time is less than T
    while timehelper.time() - t0 < T:  # go forward
        t = timehelper.time() - t0 # save time for movility
        vel = VEL #set the np array
        pos = pos + vel  * DT # get current position

        acc =np.zeros(3) # Constant velocity no acceleration
        yaw = 0.0 # set yaw
        omega = np.zeros(3) #set the array for the angular velocity

        cf.cmdFullState(pos, vel,acc, yaw, omega) #send command to the drone
        timehelper.sleepForRate(RATE)

   

    t0 = timehelper.time() #get current time to start the count
    new_pos = pos.copy() #get a copy of the current position
    while timehelper.time() - t0 < T: # Go backwards
        vel = BACK_VEL
        new_pos = new_pos + vel * DT

        acc = np.zeros(3)
        yaw = 0.0
        omega = np.zeros(3)

        cf.cmdFullState(new_pos, vel, acc, yaw, omega)
        timehelper.sleepForRate(RATE)


    cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=2.0)
    timehelper.sleep(2.5)


if __name__ == '__main__':
    main()