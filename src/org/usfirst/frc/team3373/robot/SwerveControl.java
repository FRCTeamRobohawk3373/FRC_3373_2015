package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;

public class SwerveControl  {
	
	Talon driveLFMotor;
	CANTalon rotateLFMotor;
	//Front Right Wheel
	Talon driveRFMotor;
	CANTalon rotateRFMotor;
	//Back Left Wheel
	Talon driveLBMotor;
	CANTalon rotateLBMotor;
	//Back Right Wheel
	Talon driveRBMotor;
	CANTalon rotateRBMotor;
	
	int encoderUnitsPerRotation = 1734;
	
	public SwerveControl(int frontLeftDriveChannel, int frontLeftRotateID, int frontRightDriveChannel, int frontRightRotateID, int backLeftDriveChannel, int backLeftRotateID, int backRightDriveChannel, int backRightRotateID){
		//Front Left Wheel
		driveLFMotor = new Talon(frontLeftDriveChannel);
		rotateLFMotor = new CANTalon(frontLeftRotateID);
		//Front Right Wheel
		driveRFMotor = new Talon(frontRightDriveChannel);
		rotateRFMotor = new CANTalon(frontRightRotateID);		
		//Back Left Wheel
		driveLBMotor = new Talon(backLeftDriveChannel);
		rotateLBMotor = new CANTalon(backLeftRotateID);
		//Back Right Wheel
		driveRBMotor = new Talon(backRightDriveChannel);
		rotateRBMotor = new CANTalon(backRightRotateID);
		
        rotateLFMotor.changeControlMode(CANTalon.ControlMode.Position);
        rotateLFMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
	}
	
	double deltaTheta;
	
	double proprtionalConstant = 0;
	double derivativeConstant = 0;
	double integralConstant = 0;
	
	Talon testMotor = new Talon(9);
	AnalogInput testPot = new AnalogInput(9);
	
	PIDController pid = new PIDController(proprtionalConstant, derivativeConstant, integralConstant, testPot, testMotor);
	
	public double calculateTargetDeltaTheta(int targetAngle, int currentAngle){
		double deltaThetaOne = Math.abs(targetAngle - currentAngle);
		double deltaThetaTwo = Math.abs(targetAngle - currentAngle - 180);
		
		if (deltaThetaOne <= deltaThetaTwo){
			return deltaThetaOne;//if we get here DeltaTheta must be <= 90
		} else {
			return -deltaThetaTwo;//if we get here DeltaTheta must be <= 90
		}
	}
	
    public int encoderUnitToAngle(int encoderValue){
    	System.out.println("Encoder Value: " + encoderValue);
    	int angle = 0;
    	if (encoderValue >= 0){
    		angle = (int)(encoderValue * (360.0/encoderUnitsPerRotation)) % 360;
    	} else if (encoderValue < 0){
    		angle = 360 - (int)(encoderValue * (360.0/encoderUnitsPerRotation)) % 360;
    	}
    	System.out.println(angle);
    	return angle;
    }	
	
    public int angleToEncoderUnit(int angle){//Only pass in deltaTheta
    	
    	int deltaEncoder;
    	deltaEncoder = angle*(int)(encoderUnitsPerRotation/360.0); 
    	
    	return deltaEncoder;
    }
    
	public void move(double LY, double LX, double RX){//input the target angle position for wheel, current position of wheel, Talon for the rotating motor, CANTalon for drive motor
		double radians;
		double targetTheta;
		double deltaTheta;
		double magnitude;
		
		//double deltaThetaLF;
		//double deltaThetaLB;
		//double deltaThetaRF;
		//double deltaThetaRB;
		
		radians = Math.atan2(LY, LX);
		targetTheta = Math.toDegrees(radians);
		
		if(targetTheta < 0){
			targetTheta += 360; //Get a positive equivalent angle
		}
		
		magnitude = Math.sqrt(LX*LX + LY*LY);
		
		deltaTheta = calculateTargetDeltaTheta((int)targetTheta, encoderUnitToAngle(rotateLFMotor.getEncPosition()));
		

        
        rotateLFMotor.set(rotateLFMotor.getEncPosition() + angleToEncoderUnit((int)deltaTheta));
        if (deltaTheta > 0){
        	driveLFMotor.set(magnitude);
        } else if (deltaTheta < 0){
        	driveLFMotor.set(-magnitude);
        }
		
		
	
	}
	

	
   
}

