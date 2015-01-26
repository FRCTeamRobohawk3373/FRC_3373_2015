package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveControl  {
	
	//must also use a wheel class
	
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
	
    double p = 2; //100 is very close
    double i = 0;
    double d = 5;
    //double f = 0;
    //int izone = 100;
    double ramprate = 36;
    int profile = 0;
    int drivePos = 1;
	int targetTheta;
	SwerveWheel[] wheelArray;

	SwerveWheel FLWheel;
	SwerveWheel FRWheel;
	SwerveWheel BLWheel;
	SwerveWheel BRWheel;
	
	double angleToDiagonal;
	
	/*give dimensions between the wheels both width and length, 
	 * width is the distance between left wheels and right wheels,
	 *  length is the distance between front wheels and back wheels*/
	
	public SwerveControl(int frontLeftDriveChannel, int frontLeftRotateID, int frontRightDriveChannel, 
			int frontRightRotateID, int backLeftDriveChannel, int backLeftRotateID, int backRightDriveChannel,
			int backRightRotateID, double width, double length){
		
			
		angleToDiagonal = Math.toDegrees(Math.atan2(length, width));
		
		
		FLWheel = new SwerveWheel(frontLeftDriveChannel, frontLeftRotateID, p, i, d, (270 - angleToDiagonal));
		FRWheel = new SwerveWheel(frontRightDriveChannel, frontRightRotateID, p, i, d, (angleToDiagonal + 90));
		BLWheel = new SwerveWheel(backLeftDriveChannel, backLeftRotateID, p, i, d, (angleToDiagonal + 270));
		BRWheel = new SwerveWheel(backRightDriveChannel, backRightRotateID, p, i, d, (90 - angleToDiagonal));
		
		wheelArray = new SwerveWheel[]{FLWheel, FRWheel, BLWheel, BRWheel};
	}
	
	double deltaTheta;
	
	double proprtionalConstant = .01;
	double derivativeConstant = 50;
	double integralConstant = 0;
	
	
	
	
    public int encoderUnitToAngle(int encoderValue){
    	double angle = 0;
    	if (encoderValue >= 0){
    		angle = (encoderValue * (360.0/encoderUnitsPerRotation));
    		angle = angle % 360;
    	} else if (encoderValue < 0){
    		angle = 360 - (encoderValue * (360.0/encoderUnitsPerRotation));
    		angle = angle % 360;
    	}
    	return (int)angle;
    }	
	
    public int angleToEncoderUnit(double angle){//Only pass in deltaTheta
    	
    	double deltaEncoder;
    	deltaEncoder = angle*(encoderUnitsPerRotation/360.0); 
    	
    	return (int)deltaEncoder;
    }
    
    public void calculateSwerveControl(double LY, double LX, double RX){
    	double xAxis = LX;
    	double yAxis = LY;
    	double rAxis = RX;
    	double rotateXComponent;
    	double rotateYComponent;
    	double fastestSpeed = 0;
    	
    	
    	double rotationMagnitude = Math.abs(rAxis);
    	
    	for (SwerveWheel wheel : wheelArray){
    		
    		rotateXComponent = Math.cos(Math.toRadians(wheel.rAngle)) * rotationMagnitude;
    		rotateYComponent = Math.sin(Math.toRadians(wheel.rAngle)) * rotationMagnitude;
    		
    		if(rAxis > 0){
    			rotateXComponent = -rotateXComponent;
    			rotateYComponent = -rotateYComponent;
    		}
    		
    		wheel.speed = Math.sqrt(Math.pow(rotateXComponent + xAxis, 2) + Math.pow((rotateYComponent + yAxis), 2));
    		wheel.targetAngle = Math.toDegrees(Math.atan2((rotateYComponent + yAxis), (rotateXComponent + xAxis)));
    		
    		if(wheel.speed > fastestSpeed){
    			fastestSpeed = wheel.speed;
    		}
    		
    		if(wheel.targetAngle < 0){
    			wheel.targetAngle += 360;
    		}
    		
    		wheel.currentAngle = encoderUnitToAngle(wheel.rotateMotor.getEncPosition());
    		wheel.getDeltaTheta();
    	}
    	
    	if(fastestSpeed > 1){
    		for(SwerveWheel wheel : wheelArray){
        		wheel.speed /= fastestSpeed;
        	}	
    	}
    	
    	
    	FRWheel.rotateMotor.set(FRWheel.rotateMotor.getEncPosition() - angleToEncoderUnit(FRWheel.getDeltaTheta()));
    	//FLWheel.rotateMotor.set(FLWheel.rotateMotor.getEncPosition() - angleToEncoderUnit(FLWheel.getDeltaTheta()));
    	BRWheel.rotateMotor.set(BRWheel.rotateMotor.getEncPosition() - angleToEncoderUnit(BRWheel.getDeltaTheta()));
    	BLWheel.rotateMotor.set(BLWheel.rotateMotor.getEncPosition() - angleToEncoderUnit(BLWheel.getDeltaTheta()));
    	
    	//FRWheel.driveMotor.set(FRWheel.speed);
    	//FLWheel.driveMotor.set(FRWheel.speed);
    	//BRWheel.driveMotor.set(FRWheel.speed);
    	//BLWheel.driveMotor.set(FRWheel.speed);
    	
    	
    	SmartDashboard.putNumber("Current Angle", BLWheel.currentAngle);
    	SmartDashboard.putNumber("Delta Theta", BLWheel.getDeltaTheta());
    	SmartDashboard.putNumber("Target Angle", BLWheel.targetAngle);
    
    }
    
    /*public double calculateTargetDeltaTheta(int targetAngle, int currentAngle){
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
	
	}*/
    
    
    
    /*public void move(double LY, double LX, double RX){
    	double radians;
    	double deltaTheta;
    	double magnitude;
    	
    	radians = Math.atan2(LY, LX);
    	targetTheta = (int) Math.toDegrees(radians);
    	int currentTheta = encoderUnitToAngle(rotateRBMotor.getEncPosition());
    	//targetTheta = 5000;
    	

    	
    	rotateRBMotor.set(rotateRBMotor.getEncPosition()-angleToEncoderUnit((calculateTargetDeltaTheta(targetTheta, currentTheta))));
    	SmartDashboard.putNumber("RotateLBMotor", rotateLBMotor.get());
    	SmartDashboard.putNumber("Current Theta", currentTheta);
    	SmartDashboard.putNumber("TargetTheta", calculateTargetDeltaTheta(targetTheta, currentTheta));
    	//rotateLFMotor.set(targetTheta);
    	//rotateLBMotor.set(targetTheta);
    	//rotateRFMotor.set(targetTheta);
    	//rotateRBMotor.set(targetTheta);
    	
    	//driveLFMotor.set(0.25);
    	//driveLBMotor.set(0.25);
    	//driveRFMotor.set(0.25);
    	//driveRBMotor.set(0.25);
    }	
    
    public void move(boolean a, boolean b, boolean x, boolean y){
    	double radians;
    	double deltaTheta;
    	double magnitude;
    	
    	
    	if (a){
    		targetTheta = 90;
    	} else if (b){
    		targetTheta = 360;
    	} else if (x){
    		targetTheta = 180;
    	} else if (y){
    		targetTheta = 270;
    	} 
    	int currentTheta = encoderUnitToAngle(rotateRBMotor.getEncPosition());
    	//targetTheta = 5000;
    	


    	rotateRBMotor.set(angleToEncoderUnit((calculateTargetDeltaTheta(targetTheta, currentTheta))));
    	SmartDashboard.putNumber("RotateLBMotor", rotateLBMotor.get());
    	SmartDashboard.putNumber("Current Theta", currentTheta);
    	SmartDashboard.putNumber("TargetDeltaTheta", calculateTargetDeltaTheta(targetTheta, currentTheta));
    	SmartDashboard.putNumber("TargetTheta", targetTheta);
    	SmartDashboard.putNumber("EncoderDelta", angleToEncoderUnit((calculateTargetDeltaTheta(targetTheta, currentTheta))));
    	SmartDashboard.putNumber("Encoder Position", rotateRBMotor.getEncPosition());
    }*/
    
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

