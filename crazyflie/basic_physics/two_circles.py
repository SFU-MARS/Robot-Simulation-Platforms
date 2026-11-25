from crazyflie_py import Crazyswarm
from my_demo import go_origin
from full_circle import full_circle




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

    two_circles(cf=cf, timehelper=timeHelper, R=R, z=z, period=period, steps=steps, cx=cx, cy=cy) # go and sping the world

    go_origin(cf,timeHelper=timeHelper) #go back to origin



def two_circles(cf:Crazyswarm, timehelper, R:float, z:float, steps:int, period:float, cx:float, cy:float):
    '''
    Pre:Takes drone at the initial position
    Post:Draws two cirlclesregarding the position of the origin and the drone
    '''

    full_circle(cf=cf, timehelper=timehelper, R=R, z=z, steps=steps, period=period, cx= cy, cy=cy )#Complete the circle one time
    timehelper.sleep(2.5)
    full_circle(cf=cf,timehelper=timehelper, R=R, z=z, steps=steps, period=period, cx= cx + R, cy= cy) #second Circle a bit further




if __name__ == '__main__': #call main when the function is initialied
    main()