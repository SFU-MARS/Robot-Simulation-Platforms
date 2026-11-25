from crazyflie_py import Crazyswarm
import numpy as np

Z = 1.0
TAKEOFF_DURATION = 2.5
T = 10 #set time that the simulation last
RATE = 50
R = 0.5 #radius
T_LAP = T / 2 #time to take one lap

def main():

    #Set instances
    swarm = Crazyswarm()#Create drone
    timehelper = swarm.timeHelper

    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION) #drone takes off
    timehelper.sleep(TAKEOFF_DURATION + 0.5)

    start = np.array(cf.initialPosition) + [0,0,Z] # sets up the initial position
    center = start - np.array([R, 0,0]) # get the center of the circle 
    w = 2 * np.pi /T_LAP # compute the angular velocity 

    t0 = timehelper.time() #get current time to start the count

    while timehelper.time() - t0 < T:

        t = timehelper.time() - t0 # save time for movility
        theta = t * w #Calculate the current angle

        pos = center + np.array([R * np.cos(theta), R * np.sin(theta), 0.0])
        vel = np.array([- R * w *  np.sin(theta), R *  w * np.cos(theta), 0.0]) #set the np array

        acc =np.array([-R * w **2 * np.cos(theta), - R * w ** 2 * np.sin(theta), 0.0]) # derivative of velocity, pulling towards the center
        yaw = theta + np.pi / 2 # set yaw
        omega = np.array([0,0,w]) #set the array for the angular velocity

        cf.cmdFullState(pos, vel,acc, yaw, omega) #send command to the drone, change to low level mode
        timehelper.sleepForRate(RATE)

    theta0 = np.pi #initial pi for second turn aroound, drone starts at pi
    center = start -R * np.array([np.cos(theta0), np.sin(theta0),0.0]) # get the center of the circle 
    t0 = timehelper.time()

    while timehelper.time() - t0 < T:

        t = timehelper.time() - t0 # save time for movility
        theta = theta0 + t * w #Calculate the current angle

        pos = center + np.array([R * np.cos(theta), R * np.sin(theta), 0.0])
        vel = np.array([- R * w *  np.sin(theta), R *  w * np.cos(theta), 0.0]) #set the np array

        acc =np.array([-R * w **2 * np.cos(theta), - R * w ** 2 * np.sin(theta), 0.0]) # derivative of velocity, pulling towards the center
        yaw = theta + np.pi / 2 # set yaw
        omega = np.array([0,0,w]) #set the array for the angular velocity

        cf.cmdFullState(pos, vel,acc, yaw, omega) #send command to the drone, change to low level mode
        timehelper.sleepForRate(RATE)


    cf.notifySetpointsStop() #change to high level
    cf.land(targetHeight=0.03, duration=2.0)
    timehelper.sleep(2.5)
    



if __name__ == '__main__': #call main when file is executed
    main()