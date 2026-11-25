from crazyflie_py import Crazyswarm
from my_demo import go_origin
from two_circles import two_circles
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

    infinity(cf=cf, timehelper=timeHelper, R=R, z=z, period=period, steps=steps, cx=cx, cy=cy) # go and sping the world

    go_origin(cf,timeHelper=timeHelper) #go back to origin

def infinity(cf:Crazyswarm, timehelper, R:float, z:float, period:float, steps:int, cx:float, cy:float):
    '''
    Draws the infinity symbol, or just and eight
    '''

    x0, y0 = 0.0, 0.0
    new_cx = x0 - R * np.cos(0) #Update distance from origin
    new_cy = y0 - R * np.sin(0) #Update distance form the origin

    step_time = period / steps #amount of time each step will take
    draw_circle(cf=cf, timehelper=timehelper, steps=steps, cx=new_cx, cy=new_cy, R=R, z=z, step_time=step_time, angle = 0) #DrawCricle

    new_cx = x0 - R * np.cos(np.pi) #Update distance from origin
    new_cy = y0 - R * np.sin(np.pi) #Update distance form the origin
    draw_circle(cf=cf, timehelper=timehelper, steps=steps, cx=new_cx, cy=new_cy, R=R, z=z, step_time=step_time) #DrawCricle




    

def draw_circle(cf:Crazyswarm, timehelper, steps:int, cx:float, cy:float, R:float,z:float, step_time, angle):
    '''
    Draws the circle by the given paramneters
    '''
    starting_pos = 1 if angle != 

    for i in range(1, steps + 1):
        theta = 2 * np.pi * (i / steps) #Calculate the current angle

        #Calculate new x, y, z
        x = cx + R * np.cos(theta) #calculate x, remember that cos is x in the unit circle
        y = cy + R * np.sin(theta)# calculate y, remember that is the discplacement seen form above is like y
        cf.goTo([x,y,z], yaw=0.0, duration=1, relative=False) # Move to position
        timehelper.sleep(step_time)

if __name__ == '__main__': #call main when the function is initialied
    main()