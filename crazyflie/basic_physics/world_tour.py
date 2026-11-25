from crazyflie_py import Crazyswarm
from my_demo import go_origin
import math

TARGET_HEIHGT = 1.0
TAKEOFF_DURATION = 2.5

def main():

    swarm = Crazyswarm() #Create object
    timeHelper = swarm.timeHelper #create time helper to help with timming with ROS
    cf = swarm.allcfs.crazyflies[0] #select the first element used in this case the only drone

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION) #drone takes off
    timeHelper.sleep(TAKEOFF_DURATION)

    spin_360(cf=cf, timehelper=timeHelper) # go and sping the world

    go_origin(cf,timeHelper=timeHelper) #go back to origin


def spin_360(cf: Crazyswarm, timehelper):
    '''
    Makes a full 360 degree turn
    Pre: takes and existing object and the timer
    '''
    R = 0.5
    N = 18                                 
    for i in range(1, N + 1):
        theta = i * 2 * math.pi / N
        cf.goTo([R*math.cos(theta) - R, R*math.sin(theta), 1.0],
                yaw=0.0, duration=1.5, relative=False)
        timehelper.sleep(2.0)

if __name__ == '__main__': #call main when the function is initialied
    main()

#Dynamics, input for crazyswarm