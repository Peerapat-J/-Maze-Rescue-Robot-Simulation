from cmath import nan
from email import message
from logging import root
from sqlite3 import Timestamp
from tkinter import font
from turtle import position, speed
from controller import Robot
from controller import Motor
from controller import PositionSensor
import math
import struct

"""
    Default setup
"""
robot = Robot()

# default
timeStep = 32
maxSpeed = 6.28
speed = [maxSpeed, maxSpeed]
# init motor
'''
wheel_left = robot.getMotor("Left wheel motor")
wheel_right = robot.getMotor("Right wheel motor")
wheel_left.setPosition(float('inf'))
wheel_right.setPosition(float('inf'))
'''
wheel_left = robot.getDevice("wheel1 motor")
wheel_right = robot.getDevice("wheel2 motor")
wheel_left.setPosition(float('inf'))
wheel_right.setPosition(float('inf'))

leftDistanceSensor = []
leftDistanceSensor.append(robot.getDevice("ps6"))
leftDistanceSensor[0].enable(timeStep)
leftDistanceSensor.append(robot.getDevice("ps5"))
leftDistanceSensor[1].enable(timeStep)

rightDistanceSensor = []
rightDistanceSensor.append(robot.getDevice("ps1"))
rightDistanceSensor.append(robot.getDevice("ps2"))
rightDistanceSensor[0].enable(timeStep)
rightDistanceSensor[1].enable(timeStep)

fontDistanceSensor = []
fontDistanceSensor.append(robot.getDevice("ps7"))
fontDistanceSensor.append(robot.getDevice("ps0"))
fontDistanceSensor[0].enable(timeStep)
fontDistanceSensor[1].enable(timeStep)

'''
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
'''
# get & enable color sensor
camera = robot.getCamera("camera")
camera.enable(timeStep)
camera.recognitionEnable(timeStep)

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
    #while(robot.step(timeStep) != -1):
        # if reached target velocity then terminate
    #    if left_Enc.getValue() == 0.0:
    #        break

"""
    Stop func
        Set all wheel velocity to 0.0
"""
def stop():
    wheel_left.setVelocity(0.0)
    wheel_right.setVelocity(0.0)

"""
    Forward func
        speed: velocity speed min - max = (-6.2) to 6.2
        encStep: Distance
"""
def forward():
    '''
    if speed > 5.0: speed = 5.0
    elif speed < -5.0: speed = -5.0
    if encStep is not float or encStep is not int:
        encStep = 0.0
    if encStep is nan:
        encStep = 0.0
    '''
    #global last_left_enc, last_right_enc
    #last_left_enc = left_Enc.getValue()
    #last_right_enc = left_Enc.getValue()
        
    #wheel_left.setVelocity(maxSpeed)
    #wheel_right.setVelocity(maxSpeed)
    speed[0] = maxSpeed
    speed[1] = maxSpeed
    
    # Times up check
    #while(robot.step(timeStep) != -1):
        # if reached target velocity then terminate
        #if left_Enc.getValue() > last_left_enc + encStep:
        #    break

"""
    Turn left func
        speed: velocity speed min - max = (-6.2) to 6.2
        encStep: Distance
"""
def turn_left():
    '''
    if speed > 5.0: speed = 5.0
    elif speed < -5.0: speed = -5.0
    if encStep is not float or encStep is not int:
        encStep = 0.0
    '''
    #global last_left_enc, last_right_enc
    #last_left_enc = left_Enc.getValue()
    #last_right_enc = left_Enc.getValue()
        
    #wheel_left.setVelocity(-0.4 * maxSpeed)
    #wheel_right.setVelocity(maxSpeed * 0.6)
    #stop()
    speed[0] = -0.1 * maxSpeed
    speed[1] =  maxSpeed
    
    # Times up check
    #while(robot.step(timeStep) != -1):
        # if reached target velocity then terminate
    #    if right_Enc.getValue() > last_right_enc + encStep:
    #        break

"""
    Turn right func
        speed: velocity speed min - max = (-6.2) to 6.2
        encStep: Distance
"""
def turn_right():
    '''
        if speed > 5.0: speed = 5.0
        elif speed < -5.0: speed = -5.0
        if encStep is not float or encStep is not int:
            encStep = 0.0
    ''' 
    #global last_left_enc, last_right_enc
    #last_left_enc = left_Enc.getValue()
    #last_right_enc = left_Enc.getValue()
        
    #wheel_left.setVelocity(0.6 * maxSpeed)
    #wheel_right.setVelocity(-0.4 * maxSpeed)
    #stop()
    speed[0] = maxSpeed
    speed[1] = -0.1 * maxSpeed

    # Times up check
    #while(robot.step(timeStep) != -1):
        # if reached target velocity then terminate
    #    if left_Enc.getValue() > last_left_enc + encStep:
    #        break

def spin():
    stop()
    speed[0] = (1.0 * maxSpeed)
    speed[1] = (-1.0 * maxSpeed)

def sendMessage(v1, v2, victimType):
    message = struct.pack('i i c', v1, v2, victimType.encode())
    emitter.send(message)

def sendVictimMessage(victimType = 'N'):
    global sentMessageStatus
    position = gps.getValue()
    if not sentMessageStatus:
        sendMessage(int(position[0] * 100), int(position[2] * 100), victimType)
        sentMessageStatus = True
        

    
    Normal_color = b'\xfc\xfc\xfc\xff' 
    swamp_color = b'\x8e\xde\xf5\xff'
    hole_color = b'<<<\xff'
start()
stop()
#forward_enc(5.0, 10,0) 

# speed[0]: left wheel,
# speed[1]: right wheel 


while robot.step(timeStep) != -1:
    #forward_enc(5.0, 15)
    speed[0] = maxSpeed
    speed[1] = maxSpeed

    for i in range(2):
        print("leftDistanceSensor[",i,"]", leftDistanceSensor[i].getValue())        
        print("rightDistanceSensor[",i,"]", rightDistanceSensor[i].getValue())
        print("fontDistanceSensor[",i,"]", fontDistanceSensor[i].getValue())
    stop()
    '''
    for i in range(2):
        if leftDistanceSensor[i].getValue() < 0.1:
            turn_right()
        elif rightDistanceSensor[i].getValue() < 0.2:
            turn_left()
    '''
    if fontDistanceSensor[0].getValue() < 0.1 and fontDistanceSensor[1].getValue() < 0.1:
        if leftDistanceSensor[0].getValue() < 0.1 or leftDistanceSensor[1].getValue() < 0.1:
            turn_right()

    if leftDistanceSensor[0].getValue() < 0.1 or leftDistanceSensor[1].getValue() < 0.1:
        turn_right()

    if fontDistanceSensor[0].getValue() < 0.1 and fontDistanceSensor[1].getValue() < 0.1:
        if rightDistanceSensor[0].getValue() < 0.1 or rightDistanceSensor[1].getValue() < 0.1:
            turn_left()

    if rightDistanceSensor[0].getValue() < 0.1 or rightDistanceSensor[1].getValue() < 0.1:
        turn_left()

    if fontDistanceSensor[0].getValue() < 0.1 and fontDistanceSensor[1].getValue() < 0.1:
        if leftDistanceSensor[0].getValue() < 0.1 or leftDistanceSensor[1].getValue() < 0.1:
            if rightDistanceSensor[0].getValue() < 0.1 or rightDistanceSensor[1].getValue() < 0.1:
                spin()
    
    wheel_left.setVelocity(speed[0])
    wheel_right.setVelocity(speed[1])
    