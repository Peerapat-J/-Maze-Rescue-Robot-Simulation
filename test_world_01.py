from logging import root
from controller import Robot
from controller import Motor
from controller import PositionSensor
import math
import struct

robot = Robot()

# default timing
timeStep = 32


"""
    Initialize setup
"""
wheel_left = robot.getDevice("wheel1 motor")
wheel_right = robot.getDevice("wheel2 motor")
wheel_left = setPosition('inf')
wheel_right = setPosition('inf')

left_Enc = wheel_left.getPositionSensor()
right_Enc = wheel_right.getPositionSensor()

left_Enc.enable(timeStep)
right_Enc.enable(timeStep)


last_left_enc, last_right_enc = 0.0, 0.0
"""wheel_left.setVelocity(5.0)
wheel_right.setVelocity(5.0)"""

"""
    Initial robot
"""
def start():
    
    wheel_left.setVelocity(0.0)
    wheel_right.setVelocity(0.0)

    # Times up check
    while(robot.step(timeStep) != -1):
        # if reached target velocity then terminate
        if left_Enc.getValue() == 0.0:
            break

"""
    Stop func
        Set all wheel velocity to 0.0
"""
def stop_enc():
    wheel_left.setVelocity(0.0)
    wheel_right.setVelocity(0.0)

"""
    Forward func
        speed: velocity speed min - max = 5 to -5
        encStep: Distance
"""
def forward_enc(speed, encStep):
    if speed > 5.0: speed = 5.0
    elif speed < -5.0: speed = -5.0
    if encStep is not float or encStep is not int:
        encStep = 0.0
    
    global last_left_enc, last_right_enc
    last_left_enc = left_Enc.getValue()
    last_right_enc = left_Enc.getValue()
        
    wheel_left.setVelocity(speed)
    wheel_right.setVelocity(speed)

    # Times up check
    while(robot.step(timeStep) != -1):
        # if reached target velocity then terminate
        if left_Enc.getValue() > last_left_enc + encStep:
            break


"""
    Turn left func
        speed: velocity speed min - max = 5 to -5
        encStep: Distance
"""
def turnLeft_enc(speed, encStep):
    if speed > 5.0: speed = 5.0
    elif speed < -5.0: speed = -5.0
    if encStep is not float or encStep is not int:
        encStep = 0.0
    
    global last_left_enc, last_right_enc
    last_left_enc = left_Enc.getValue()
    last_right_enc = left_Enc.getValue()
        
    wheel_left.setVelocity(-speed)
    wheel_right.setVelocity(speed)

    # Times up check
    while(robot.step(timeStep) != -1):
        # if reached target velocity then terminate
        if left_Enc.getValue() > last_right_enc + encStep:
            break


forward_enc(5.0, 10,0)
stop_enc()


