#This is the math behind the swerve drive in Python 2.7

import platform
import math

def main():
    print("Currently running in Python " + platform.python_version())

    #Dummy values for testing. These will be read from the controller in the robot code
    xAxis = math.sqrt(2) / 2 #Where -1 is left and 1 is right
    yAxis = math.sqrt(2) / 2 #Where -1 is back and 1 is forward
    rAxis = 1 #Where -1 i counterclockwise and 1 is clockwise
    orientationOffset = 270

    #Constants
    wheelWidth = 1 #Perpenducilar to the forward direction(Distance between the wheels)
    wheelLength = 1 #Parallel to the forward direction
    rotateAngle = math.degrees(math.atan2(wheelLength, wheelWidth)) #This is the ratio of the width divided by length of the wheel position

    #Declare Wheels and Assign Each an Angle of Rotation
    frontLeftWheel = Wheel(270 - rotateAngle, "Front Left") #Front Left
    backLeftWheel = Wheel(rotateAngle + 270, "Back Left") #Back Left
    backRightWheel = Wheel(90 - rotateAngle, "Back Right") #Back Right
    frontRightWheel = Wheel(rotateAngle + 90, "Front Right") #Front Right

    rotationMagnitude = abs(rAxis)

    #Split Translation Vector
    translationalXComponent = xAxis
    translationalYComponent = yAxis

    translationalMagnitude = math.sqrt(math.pow(translationalYComponent, 2) + math.pow(translationalXComponent, 2))
    translationalAngle = math.degrees(math.atan2(translationalYComponent, translationalXComponent))

    #Add Offset to Translation Vector
    translationalAngle += orientationOffset
    if translationalAngle < 0: translationalAngle += 360
    if translationalAngle >= 360: translationalAngle -= 360

    #Recombine New Translation Vector
    translationalYComponent = math.sin(math.radians(translationalAngle)) * translationalMagnitude
    translationalXComponent = math.cos(math.radians(translationalAngle)) * translationalMagnitude

    fastest = 0
    for wheel in (frontLeftWheel, backLeftWheel, backRightWheel, frontRightWheel):
        #Get Rotational Components for Each Wheel
        rotationalXComponent = math.cos(math.radians(wheel.rotateAngle)) * rotationMagnitude
        rotationalYComponent = math.sin(math.radians(wheel.rotateAngle)) * rotationMagnitude

        #Assign Direction (Positive is Clockwise)
        if rAxis > 0:
            rotationalXComponent = -rotationalXComponent
            rotationalYComponent = -rotationalYComponent

        #Add Rotational and Translational Vectors
        wheel.speed = math.sqrt(math.pow((rotationalXComponent + translationalXComponent), 2) + math.pow((rotationalYComponent + translationalYComponent), 2))
        wheel.angle = math.degrees(math.atan2((rotationalYComponent + translationalYComponent), (rotationalXComponent + translationalXComponent)))

        #Ensure Values are in Valid Range
        if wheel.angle < 0: wheel.angle += 360
        if wheel.angle >= 360: wheel.angle -= 360

        #Tracks Fastest Wheel
        if wheel.speed > fastest:
            fastest = wheel.speed

    #Slows Wheels Down to Maximum Speed
    for wheel in (frontLeftWheel, backLeftWheel, backRightWheel, frontRightWheel):
        if fastest > 1:
            wheel.speed /= fastest

        print wheel.pos + " Angle: " + str(wheel.angle)
        print wheel.pos + " Speed: " + str(wheel.speed)

#This class represents a swerve drive module
class Wheel:
    def __init__(self, rotateAngle, pos):
        self.pos = pos
        self.speed = 0
        self.angle = 0
        self.rotateAngle = rotateAngle

main()