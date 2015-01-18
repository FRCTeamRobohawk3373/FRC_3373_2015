package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;

public class SwerveControl  {
	
	public SwerveControl(int frontLeftDriveID, int frontLeftRotateChannel, int frontRightDriveID, int frontRightRotateChannel, int backLeftDriveID, int backLeftRotateChannel, int backRightDriveID, int backRightRotateChannel){
		//Front Left Wheel
		CANTalon driveLFMotor = new CANTalon(frontLeftDriveID);
		Talon rotateLFMotor = new Talon(frontLeftRotateChannel);
		//Front Right Wheel
		CANTalon driveRFMotor = new CANTalon(frontRightDriveID);
		Talon rotateRFMotor = new Talon(frontRightRotateChannel);		
		//Back Left Wheel
		CANTalon driveLBMotor = new CANTalon(backLeftDriveID);
		Talon rotateLBMotor = new Talon(backLeftRotateChannel);
		//Back Right Wheel
		CANTalon driveRBMotor = new CANTalon(backRightDriveID);
		Talon rotateRBMotor = new Talon(backRightRotateChannel);
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
	
	public void move(double LY, double LX, double RX){//input the target angle position for wheel, current position of wheel, Talon for the rotating motor, CANTalon for drive motor
		double radians;
		double theta;
		
		if(LX < 0){
			radians = Math.atan2(LY, LX);
			theta = Math.toDegrees(radians);
			theta = theta + 2*(90-theta);
		} else{
			radians = Math.atan2(LY, LX);
			theta = Math.toDegrees(radians);
			if(theta < 0){ //If the angle is negative, add 360 to get a positive equivalent value
				theta += 360;
			}
		}
		
		pid.enable();
		pid.setInputRange(0, 359);
		pid.setOutputRange(-1, 1);
		pid.setPercentTolerance(0.1);		
		
		
		pid.setSetpoint(theta);
		pid.getError();
		
	
	}
	/*
	public int encoderValueToAngle(int encoderValue){//input value of encoder and get an angle based off that
		
		//DO math here
		return angle;
	}*/
}

