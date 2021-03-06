from cmath import nan
from email import message
from logging import root
from sqlite3 import Timestamp
from turtle import position
from controller import Robot
from controller import Motor
from controller import PositionSensor
import math
import struct

"""
    Default setup
"""
robot = Robot()

# default timing
timeStep = 32

# init motor
wheel_left = robot.getDevice("wheel1 motor")
wheel_right = robot.getDevice("wheel2 motor")
wheel_left.setPosition(float('inf'))
wheel_right.setPosition(float('inf'))

# init sensor ps1 to ps7
ps0 = robot.getDevice("ps0")
ps1 = robot.getDevice("ps1")
ps2 = robot.getDevice("ps2")
ps3 = robot.getDevice("ps3")
ps4 = robot.getDevice("ps4")
ps5 = robot.getDevice("ps5")
ps6 = robot.getDevice("ps6")
ps7 = robot.getDevice("ps7")
# enable all
ps0.enable(timeStep)
ps1.enable(timeStep)
ps2.enable(timeStep)
ps3.enable(timeStep)
ps4.enable(timeStep)
ps5.enable(timeStep)
ps6.enable(timeStep)
ps7.enable(timeStep)

# get & enable color sensor
colorSensor = robot.getCamera("color_sensor")
colorSensor.enable(timeStep)

# get & enable emitter for 
# no need to enable since not always triggered
emitter = robot.getEmitter("emitter")

gps = robot.getGPS("gps")
gps.enable(timeStep)

sentMessageStatus = False

# init encoder
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
        speed: velocity speed min - max = (-6.2) to 6.2
        encStep: Distance
"""
def forward_enc(speed, encStep):
    '''
    if speed > 5.0: speed = 5.0
    elif speed < -5.0: speed = -5.0
    if encStep is not float or encStep is not int:
        encStep = 0.0
    if encStep is nan:
        encStep = 0.0
    '''
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
        speed: velocity speed min - max = (-6.2) to 6.2
        encStep: Distance
"""
def turn_left_enc(speed, encStep):
    '''
    if speed > 5.0: speed = 5.0
    elif speed < -5.0: speed = -5.0
    if encStep is not float or encStep is not int:
        encStep = 0.0
    '''
    global last_left_enc, last_right_enc
    last_left_enc = left_Enc.getValue()
    last_right_enc = left_Enc.getValue()
        
    wheel_left.setVelocity(-speed)
    wheel_right.setVelocity(speed)

    # Times up check
    while(robot.step(timeStep) != -1):
        # if reached target velocity then terminate
        if right_Enc.getValue() > last_right_enc + encStep:
            break

"""
    Turn right func
        speed: velocity speed min - max = (-6.2) to 6.2
        encStep: Distance
"""
def turn_right_enc(speed, encStep):
    '''
        if speed > 5.0: speed = 5.0
        elif speed < -5.0: speed = -5.0
        if encStep is not float or encStep is not int:
            encStep = 0.0
    ''' 
    global last_left_enc, last_right_enc
    last_left_enc = left_Enc.getValue()
    last_right_enc = left_Enc.getValue()
        
    wheel_left.setVelocity(speed)
    wheel_right.setVelocity(-speed)

    # Times up check
    while(robot.step(timeStep) != -1):
        # if reached target velocity then terminate
        if left_Enc.getValue() > last_left_enc + encStep:
            break

def sendMessage(v1, v2, victimType):
    message = struct.pack('i i c', v1, v2, victimType.encode())
    emitter.send(message)

def sendVictimMessage(victimType = 'N'):
    global sentMessageStatus
    position = gps.getValue()
    if not sentMessageStatus:
        sendMessage(int(position[0] * 100), int(position[2] * 100), victimType)
        sentMessageStatus = True
        

    

start()
stop_enc()
#forward_enc(5.0, 10,0) 
while robot.step(timeStep) != -1:
    forward_enc(5.0, 15)
    
    '''
        print("ps0: ", ps0.getValue())
        print("ps1: ", ps0.getValue())
        print("ps2: ", ps0.getValue())
        print("ps3: ", ps0.getValue())
        print("ps4: ", ps0.getValue())
        print("ps5: ", ps0.getValue())
        print("ps6: ", ps0.getValue())
        print("ps7: ", ps0.getValue())
    '''
    if ps7.getValue() < 0.044:
        stop_enc()
        if ps5.getValue() < 0.06:
            turn_right_enc(5.0, 2.1)
            stop_enc()
            wheel_left.setVelocity(5.0)
            wheel_right.setVelocity(5.0)
            

    Normal_color = b'\xfc\xfc\xfc\xff' 
    swamp_color = b'\x8e\xde\xf5\xff'
    hole_color = b'<<<\xff'
 
    print(colorSensor.getImage())