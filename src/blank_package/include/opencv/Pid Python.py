#PID PYTHON

import numpy as np
import time
from opencv_functions import Image
import os
import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from duckietown_msgs.msg import WheelsCmdStamped
from rclpy.time import Duration


# img = Image()
# img.find_error_from_middle


def pid_controller(kp, ki, kd, previous_error, integral, dt, image_obj):
    percentage_error, error = image_obj.find_error_from_middle() # PV is the real-time measured value of the system parameter i want to control, (In my case this will be distance from center i think) here it is subtracting the Process Variable from the desired set point to find the real-time error
    integral += error * dt # Integral here is adding the errors every time step (i think) the time step is represented by dt. As the integral term accumulates over time, the 'urge' to return the error to zero increases.
    derivative = (error - previous_error) / dt # Looks like acceleration formula!! rate of change of error
    control = kp * error + ki * integral + kd * derivative
    return control, error, integral

def main():
    kp = 1.55 # Proportional Gain (determines how strongly the output reacts to the current error by agressively correcting)
    ki = 0.0 # Integral Gain (determines how agressively the controller ramps up the correction when the error won't go away)
    kd = 0.50 # Derivative Gain (determines how strong the preventative correction value will be)
    previous_error = 0
    integral = 0
    dt = 0.1 # Time step

    time_steps = [] # Stores time points at each iteration
    control_values = [] # Stores control output at each time step

# if

    

   

if __name__ == "__main__":
        main()
