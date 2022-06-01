from cmath import nan, pi
from email import message
from logging import root
from sqlite3 import Timestamp
from tkinter import font
from turtle import position, speed
#import cv2
#import numpy as np
from controller import Robot
from controller import Motor
from controller import PositionSensor
import math
import struct
robot = Robot()

# default value
timeStep = 32
maxVelocity = 6.28

# color definition for trap detect
#normPlate = b'\xfc\xfc\xfc\xff' 
swampColor = b'\x8e\xde\xf5\xff'   #swamp_color = b'R\x89\xa7\xff'
holeColor = b'<<<\xff'             #hole_color = b'\x1e\x1e\x1e\xff'

# for victim identification
distanceFromWall = 0.05
victimProximity = 0.03
messageSentTrigger = False

startTime = 0
duration = 0
victimDetectedCheck = False
victimTimer = 0

speed = [maxVelocity, maxVelocity]
useCamera = True
# init motor
'''
wheelLeft = robot.getMotor("Left wheel motor")
wheelRight = robot.getMotor("Right wheel motor")
wheelLeft.setPosition(float('inf'))
wheelRight.setPosition(float('inf'))
'''
wheelLeft = robot.getDevice("wheel1 motor")
wheelRight = robot.getDevice("wheel2 motor")
wheelLeft.setPosition(float('inf'))
wheelRight.setPosition(float('inf'))

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
camera = robot.getDevice("colour_sensor")
camera.enable(timeStep)
camera.recognitionEnable(timeStep)

# get & enable emitter for 
# no need to enable since not always triggered
emitter = robot.getEmitter("emitter")

gps = robot.getGPS("gps")
gps.enable(timeStep)

sentMessageStatus = False

# init encoder
left_Enc = wheelLeft.getPositionSensor()
right_Enc = wheelRight.getPositionSensor()
left_Enc.enable(timeStep)
right_Enc.enable(timeStep)

last_left_enc, last_right_enc = 0.0, 0.0
"""wheelLeft.setVelocity(5.0)
wheelRight.setVelocity(5.0)"""

"""
    Initial robot
"""
def start():
    
    wheelLeft.setVelocity(0.0)
    wheelRight.setVelocity(0.0)

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
    wheelLeft.setVelocity(0.0)
    wheelRight.setVelocity(0.0)

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
        
    #wheelLeft.setVelocity(maxVelocity)
    #wheelRight.setVelocity(maxVelocity)
    speed[0] = maxVelocity
    speed[1] = maxVelocity
    
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
def turnLeft():
    '''
    if speed > 5.0: speed = 5.0
    elif speed < -5.0: speed = -5.0
    if encStep is not float or encStep is not int:
        encStep = 0.0
    '''
    #global last_left_enc, last_right_enc
    #last_left_enc = left_Enc.getValue()
    #last_right_enc = left_Enc.getValue()
        
    #wheelLeft.setVelocity(-0.4 * maxVelocity)
    #wheelRight.setVelocity(maxVelocity * 0.6)
    #stop()
    speed[0] = -0.1 * maxVelocity
    speed[1] =  maxVelocity
    
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
def turnRight():
    '''
        if speed > 5.0: speed = 5.0
        elif speed < -5.0: speed = -5.0
        if encStep is not float or encStep is not int:
            encStep = 0.0
    ''' 
    #global last_left_enc, last_right_enc
    #last_left_enc = left_Enc.getValue()
    #last_right_enc = left_Enc.getValue()
        
    #wheelLeft.setVelocity(0.6 * maxVelocity)
    #wheelRight.setVelocity(-0.4 * maxVelocity)
    #stop()
    speed[0] = maxVelocity
    speed[1] = -0.1 * maxVelocity

    # Times up check
    #while(robot.step(timeStep) != -1):
        # if reached target velocity then terminate
    #    if left_Enc.getValue() > last_left_enc + encStep:
    #        break

def spin():
    stop()
    speed[0] = (1.0 * maxVelocity)
    speed[1] = (-1.0 * maxVelocity)

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
stop()
#forward_enc(5.0, 10,0) 

# speed[0]: left wheel,
# speed[1]: right wheel 


while robot.step(timeStep) != -1:
    #forward_enc(5.0, 15)
    speed[0] = maxVelocity
    speed[1] = maxVelocity

    for i in range(2):
        print("leftDistanceSensor[",i,"]", leftDistanceSensor[i].getValue())        
        print("rightDistanceSensor[",i,"]", rightDistanceSensor[i].getValue())
        print("fontDistanceSensor[",i,"]", fontDistanceSensor[i].getValue())
    stop()
    '''
    for i in range(2):
        if leftDistanceSensor[i].getValue() < 0.1:
            turnRight()
        elif rightDistanceSensor[i].getValue() < 0.2:
            turnLeft()
    '''
    if fontDistanceSensor[0].getValue() < 0.1 and fontDistanceSensor[1].getValue() < 0.1:
        if leftDistanceSensor[0].getValue() < 0.1 or leftDistanceSensor[1].getValue() < 0.1:
            turnRight()

    if leftDistanceSensor[0].getValue() < 0.1 or leftDistanceSensor[1].getValue() < 0.1:
        turnRight()

    if fontDistanceSensor[0].getValue() < 0.1 and fontDistanceSensor[1].getValue() < 0.1:
        if rightDistanceSensor[0].getValue() < 0.1 or rightDistanceSensor[1].getValue() < 0.1:
            turnLeft()

    if rightDistanceSensor[0].getValue() < 0.1 or rightDistanceSensor[1].getValue() < 0.1:
        turnLeft()

    if fontDistanceSensor[0].getValue() < 0.1 and fontDistanceSensor[1].getValue() < 0.1:
        if leftDistanceSensor[0].getValue() < 0.1 or leftDistanceSensor[1].getValue() < 0.1:
            if rightDistanceSensor[0].getValue() < 0.1 or rightDistanceSensor[1].getValue() < 0.1:
                spin()
    
    wheelLeft.setVelocity(speed[0])
    wheelRight.setVelocity(speed[1])
    