package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	
	int encoderUnitsPerRotation = 1665;
	
    double p = 10; //100 is very close
    double i = 0;
    double d = 0;
    double f = 0;
    int izone = 100;
    double ramprate = 36;
    int profile = 0;
    int drivePos = 1;
	double targetTheta;

	
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
        
		rotateLFMotor.setPID(p,i,d);
        rotateLFMotor.changeControlMode(CANTalon.ControlMode.Position);
        rotateLFMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        targetTheta = rotateLFMotor.getEncPosition();
	}
	
	double deltaTheta;
	
	double proprtionalConstant = 0;
	double derivativeConstant = 0;
	double integralConstant = 0;
	
	
	public double calculateTargetDeltaTheta(int targetAngle, int currentAngle){
		double deltaThetaOne; 
		double deltaThetaTwo; 
		double deltaThetaOne360;
		double deltaThetaTwo360;
		double deltaThetaOneTarget;
		double deltaThetaTwoTarget;
		
		if (currentAngle == 0){
			deltaThetaOneTarget = Math.abs(targetAngle - 360);
			deltaThetaTwoTarget = Math.abs(targetAngle - 540);
		} else {
			deltaThetaOneTarget = 1000;
			deltaThetaTwoTarget = 1000;
		}
		
		if (targetAngle == 0){
			deltaThetaOne360 = Math.abs(360-currentAngle);
			deltaThetaTwo360 = Math.abs(360 - currentAngle - 180);
		} else {
			deltaThetaOne360 = 1000;
			deltaThetaTwo360 = 1000;
		}


		deltaThetaOne = Math.abs(targetAngle - currentAngle);
		deltaThetaTwo = Math.abs(targetAngle - currentAngle - 180);
		
		SmartDashboard.putNumber("DeltaThetaOne: ", deltaThetaOne);
		SmartDashboard.putNumber("DeltaThetaTwo: ", deltaThetaTwo);
		SmartDashboard.putNumber("DeltaThetaOne360: ", deltaThetaOne360);
		SmartDashboard.putNumber("DeltaThetaTwo360: ", deltaThetaTwo360);
		SmartDashboard.putNumber("DeltaThetaOneTarget: ", deltaThetaOneTarget);
		SmartDashboard.putNumber("DeltaThetaTwoTarget: ", deltaThetaTwoTarget);
		
		if (deltaThetaOne <= 90){
			return -deltaThetaOne;//if we get here DeltaTheta must be <= 90
		} else if (deltaThetaTwo <= 90){
			return deltaThetaTwo;//if we get here DeltaTheta must be <= 90
		} else if (deltaThetaOne360 <= 90){
			return -deltaThetaOne360;
		} else if (deltaThetaTwo360 <= 90){
			return deltaThetaTwo360;
		} else if (deltaThetaOneTarget <= 90){
			return -deltaThetaOneTarget;
		} else if (deltaThetaTwoTarget <= 90){
			return deltaThetaTwoTarget;
		} else return 0;
		
	}
	
    public int encoderUnitToAngle(int encoderValue){
    	int angle = 0;
    	if (encoderValue >= 0){
    		angle = (int)((encoderValue * (360.0/encoderUnitsPerRotation)) % 360);
    	} else if (encoderValue < 0){
    		angle = 360 - (int)((encoderValue * (360.0/encoderUnitsPerRotation)) % 360);
    	}
    	return angle;
    }	
	
    public int angleToEncoderUnit(int angle){//Only pass in deltaTheta
    	
    	int deltaEncoder;
    	deltaEncoder = angle*(int)(encoderUnitsPerRotation/360.0); 
    	
    	return deltaEncoder;
    }
    
    public void move(double LY, double LX, double RX){
    	double radians;
    	double deltaTheta;
    	double magnitude;
    	
    	radians = Math.atan2(LY, LX);
    	targetTheta = Math.toDegrees(radians);
    	
    	targetTheta = 5000;
    	

    	
    	rotateLFMotor.set(targetTheta);
    	//rotateLBMotor.set(targetTheta);
    	//rotateRFMotor.set(targetTheta);
    	//rotateRBMotor.set(targetTheta);
    	
    	driveLFMotor.set(0.25);
    	//driveLBMotor.set(0.25);
    	//driveRFMotor.set(0.25);
    	//driveRBMotor.set(0.25);
    }	
    
    /*
	public void move(double LY, double LX, double RX){//input the target angle position for wheel, current position of wheel, Talon for the rotating motor, CANTalon for drive motor
		double radians;
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
        //if (deltaTheta > 0){
        //	driveLFMotor.set(magnitude);
        //} else if (deltaTheta < 0){
        //	driveLFMotor.set(-magnitude);
        //}
		
		SmartDashboard.putNumber("Target Angle: ", (int)targetTheta);
		SmartDashboard.putNumber("Delta Theta: ", deltaTheta);
		SmartDashboard.putNumber("Target Change", rotateLFMotor.getEncPosition() + angleToEncoderUnit((int)deltaTheta));
        SmartDashboard.putNumber("Current Encoder", rotateLFMotor.getEncPosition());
        SmartDashboard.putNumber("Current Angle", encoderUnitToAngle(rotateLFMotor.getEncPosition()));
		
	}*/
	

	
   
}

