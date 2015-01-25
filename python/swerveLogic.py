#This is test code for the GitHub Repository
DEBUG = True

import platform
import math

def main():
    print("Currently running in Python " + platform.python_version())
    
    #Dummy values for testing. These will be read from the controller in the robot code
    xAxis = math.sqrt(2)/2 #Where -1 is left and 1 is right
    yAxis = math.sqrt(2)/2 #Where -1 is back and 1 is forward
    rAxis = 1 #Where -1 i counterclockwise and 1 is clockwise
    
    #Constants
    wheelWidth = 1 #Perpenducilar to the foward direction(Distance between the wheels)
    wheelLength = 1 #Parallel to the forward direction
    rotateAngle = math.degrees(math.atan2(wheelLength, wheelWidth)) #This is the ratio of the width divided by length of the wheel position
    
    #Declare Wheels
    wheelA = Wheel(0, 0, rotateAngle + 180) #Front Left
    wheelB = Wheel(0, 0, rotateAngle + (360 - 2 * rotateAngle)) #Back Left
    wheelC = Wheel(0, 0, rotateAngle) #Back Right
    wheelD = Wheel(0, 0, rotateAngle + (180 - 2 * rotateAngle)) #Front Right

    rotationMagnitude = abs(rAxis)

    largest = 0

    for wheel in (wheelA, wheelB, wheelC, wheelD):
        #Sum Wheel Vectors
        rotateX = math.cos(math.radians(wheel.rotateAngle)) * rotationMagnitude
        rotateY = math.sin(math.radians(wheel.rotateAngle)) * rotationMagnitude
    
        if rAxis < 0:
            rotateX = -rotateX
            rotateY = -rotateY
        
        wheel.speed = math.sqrt(math.pow((rotateX + xAxis), 2) + math.pow((rotateY + yAxis), 2))
        wheel.angle = math.degrees(math.atan2((rotateY + yAxis), (rotateX + xAxis)))
        
        if wheel.speed > largest:
            largest = wheel.speed
        
        if wheel.angle < 0:
            wheel.angle += 360
    
    for wheel in (wheelA, wheelB, wheelC, wheelD):
        wheel.speed /= largest
        debugPrint(wheel.angle)
        debugPrint(wheel.speed)

def debugPrint(string):
    if DEBUG:
        print(string)

class Wheel:
    def __init__(self, speed, angle, rotateAngle):
        self.speed = speed
        self.angle = angle
        self.rotateAngle = rotateAngle
        
main()