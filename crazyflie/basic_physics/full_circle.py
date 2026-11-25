from crazyflie_py import Crazyswarm
from my_demo import go_origin
import numpy as np

TARGET_HEIHGT = 1.0
TAKEOFF_DURATION = 2.5



def main():

    swarm = Crazyswarm() #Create object
    timeHelper = swarm.timeHelper #create time helper to help with timming with ROS
    cf = swarm.allcfs.crazyflies[0] #select the first element used in this case the only drone
   
    #Define the constants
    R = 0.5 #radius
    z = 1.5 #Height z -> height
    period = 10.0 #Time for a full cirlce in seconds
    steps = 50 # number of steps that the drone will take
    position = cf.position #get the position
    cx = position[0] # get origin x -> forwards if negative is backwards
    cy = position[1] # get origin y -> left is positive, rigth if negative
    
    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION) #drone takes off
    timeHelper.sleep(TAKEOFF_DURATION)

    full_circle(cf=cf, timehelper=timeHelper, R=R, z=z,period=period, steps=steps,cx=cx, cy=cy) # go and sping the world

    go_origin(cf,timeHelper=timeHelper) #go back to origin



def full_circle(cf: Crazyswarm, timehelper, R, z, period, steps, cx, cy):
    '''
    Pre: takes a drone object and a timer
    Post: the drones makes full circle
    '''

    cf.goTo([cx + R, cy, z ], yaw=0.0, duration=2.5, relative=False) #move to the radius
    timehelper.sleep(2.5)

    step_time = period / steps #amount of time each step will take


    for i in range(1, steps + 1):
        theta = 2 * np.pi * (i / steps) #Calculate the current angle

        #Calculate new x, y, z
        x = cx + R * np.cos(theta) #calculate x, remember that cos is x in the unit circle
        y = cy + R * np.sin(theta)# calculate y, remember that is the discplacement seen form above is like y
        cf.goTo([x,y,z], yaw=0.0, duration=1, relative=False) # Move to position
        timehelper.sleep(step_time)

    cf.goTo([cx -cx, cy, z ], yaw=0.0, duration=2.5, relative=False) #move back to the origin

    



if __name__ == '__main__': #call main when the function is initialied
    main()