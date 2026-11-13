#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim

import os
import select
import json
import sys
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""



def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output_vel, input_vel, slop):
    if input_vel > output_vel:
        output_vel = min(input_vel, output_vel + slop)
    elif input_vel < output_vel:
        output_vel = max(input_vel, output_vel - slop)
    else:
        output_vel = input_vel

    return output_vel


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)

def publish_command(pub, linear_vel=0.0, angular_vel=0.0):
    # Send angular velocities and linear velocities to the robot
    # target_linear_velocity = hj_results.linear.x
    # target_angular_velocity = hj_results.angular.z
    control_linear_velocity = linear_vel
    control_angular_velocity = angular_vel
    twist = Twist()
    twist.linear.x = control_linear_velocity 
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = control_angular_velocity

    print("Publishing command: linear velocity = {}, angular velocity = {}".format(control_linear_velocity, control_angular_velocity))
    pub.publish(twist)

update_index = 0

class TimerController(Node):
    def __init__(self):
        super().__init__('controller_node')
       
        qos = QoSProfile(depth=10)
        pub = self.create_publisher(Twist, 'cmd_vel', qos)
        print(os.getcwd())
        with open('turtlebot3_controls/turtlebot3_controls/script/dubins_control.txt', 'r') as f:
            optimal_controls = list(map(float, f.readlines()))

        with open('turtlebot3_controls/turtlebot3_controls/script/robot_config.json', 'r') as f:
            config = json.load(f)
            velocity = config["velocity"]
            t_step = config["t_step"]

        self.pub = pub
        self.optimal_controls = optimal_controls
        self.velocity = velocity
        self.update_index = 0

        self.timer = self.create_timer(
            t_step,
            self.update_robot
        )

    def update_robot(self):
        print(self.update_index, len(self.optimal_controls))
        if self.update_index < len(self.optimal_controls):
            angular_vel = self.optimal_controls[self.update_index]
            publish_command(self.pub, linear_vel=self.velocity, angular_vel=float(angular_vel))
            self.update_index += 1
        elif self.update_index < len(self.optimal_controls):
            pass
        else:
            print("DONE")
            publish_command(self.pub, 0.0, 0.0)  # Send a final command to stop the robot
            exit(0)
            self.timer.cancel()
            rclpy.shutdown()  # Wake spin() and let main() clean up the node

# def update_robot(pub, optimal_controls, velocity):
#     global update_index
#     if update_index < len(optimal_controls):
#         angular_vel = optimal_controls[update_index]
#         publish_command(pub, linear_vel=velocity, angular_vel=float(angular_vel))
#         update_index += 1
#     else:
#         publish_command(pub, 0.0, 0.0)  # Send a final command to stop the robot
#         self.timer.cancel()  # Stop the timer
#         rclpy.shutdown()     # Unblock spin and signal shutdown


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TimerController()

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    print("Starting the control loop. Press Ctrl+C to exit.")
    
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Main thread interrupted.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
        


 


if __name__ == '__main__':
    main()
