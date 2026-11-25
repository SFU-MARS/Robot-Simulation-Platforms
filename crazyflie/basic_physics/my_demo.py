
from crazyflie_py import Crazyswarm

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
LAND_DURATION = 2.5



def main():
    swarm:Crazyswarm = Crazyswarm() #Create swarm object
    TimeHelper= swarm.timeHelper #create the time helper to control timming

    cf = swarm.allcfs.crazyflies[0] # get configurations fro the drone commands

    #Drone takes off

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION) #make the drone move infront
    TimeHelper.sleep(TAKEOFF_DURATION )

    #Go really far
    go_far(cf, 3,timeHelper=TimeHelper) #move .5 x each distance

    go_origin(cf, timeHelper=TimeHelper)
    

    cf.land(targetHeight= 0.0, duration=3)

def go_origin(cf:Crazyswarm, timeHelper):
    '''
    Drone goes back to the origin
    Pre: takes an object swarm
    Post: the drone goes back to the origin after function being executed
    '''

    cf.goTo(goal=[0.0,0.0,1.0], relative=False, duration=4, yaw=0) #make tdrone go back to origin
    timeHelper.sleep(4)
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3.0)

def go_far(cf:Crazyswarm, distance:int,timeHelper):
    '''
    Will make the drone go really far
    Pre: Takes a cf configuration of the object to call the different functions
    Post: The drone will be really really really far away
    '''
    for i in range(distance):
        cf.goTo(goal=[1.0, 0.0,0.0], relative=True, duration=2.0, yaw= 0) #make the drone move far
        timeHelper.sleep(2.0)


if __name__ =='__main__':
    main()



    