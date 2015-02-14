package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveWheel {

	public Talon driveMotor; //private Talon driveMotor;
	private CANTalon rotateMotor;
	
	private double rAngle;
	private double speed;
	private int targetAngle;
	private int encoderUnitsPerRotation = 1660;//was 1665
	private double speedModifier = 0.5;
	private int encoderAtHome = 0;
	private int homeToZero = 0;

	
	//orientation is negative 1, if banebots are facing forwards, orientation is 1 if the banebots are facing backwards
	public SwerveWheel(int driveMotorChannel, int rotateMotorID, double p, double i, double d, double rotateAngle, int distanceFromZero){
		
		driveMotor = new Talon(driveMotorChannel);
		rotateMotor = new CANTalon(rotateMotorID);
		
		rotateMotor.setPID(p,i,d);
        rotateMotor.changeControlMode(CANTalon.ControlMode.Position);
        rotateMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rotateMotor.enableLimitSwitch(false, false);
        rotateMotor.enableBrakeMode(true);
        
        targetAngle = encoderUnitToAngle(rotateMotor.getEncPosition());
		rAngle = rotateAngle;
		homeToZero = distanceFromZero;
	}
	
	public double getDeltaTheta(){
		double deltaTheta = getTargetAngle() - getCurrentAngle();
		
		while ((deltaTheta < -90) || (deltaTheta > 90)){
			if(deltaTheta > 90){
				deltaTheta -= 180;
				speed *= -1;
			}else if(deltaTheta < -90){
				deltaTheta += 180;
				speed *= -1;
			}
		}
		
		//if (deltaTheta >= 1 || deltaTheta <= -1){
			return deltaTheta;
		//} else {
		//	return 0;
		//}
		
	}
	
	public void setTargetAngle(double angle){
		
		//angle += orientationOffset; this does not work
		
		if(angle < 0){
			angle += 360;
		}else if(angle >=360){
			angle -= 360;
		}
		
		targetAngle = (int)angle ;
	}
	
	public int getTargetAngle(){
		return targetAngle;
	}
	
	public int getCurrentAngle(){
		//system.out.println(encoderUnitToAngle(getEncoderValue()));
		return encoderUnitToAngle(getEncoderValue());
		//return encoderUnitToAngle(rotateMotor.getEncPosition());
	}
	
	public int getRAngle(){
		return (int)rAngle;
	}
	
	public void goToAngle(){
		rotateMotor.set(getEncoderValue() + angleToEncoderUnit(getDeltaTheta()));
		//rotateMotor.set(rotateMotor.getEncPosition() + angleToEncoderUnit(getDeltaTheta()));
	}
	
	public void setSpeed(double magnitude){
		speed = magnitude;
	}
	public double getSpeed(){
		return speed;
	}
	public int getEncoderValue(){
		return rotateMotor.getEncPosition();// - homeToZero;
	}

	
	
	
	
	
    public int encoderUnitToAngle(int encoderValue){
    	double angle = 0;
    	if (encoderValue >= 0){
    		angle = (encoderValue * (360.0/encoderUnitsPerRotation));
    		angle = angle % 360;
    	} else if (encoderValue < 0){
    		angle = (encoderValue * (360.0/encoderUnitsPerRotation));
    		angle = angle % 360 + 360;
    	}
    	return (int)angle;//(angle+2*(90-angle));
    }
    
    public int angleToEncoderUnit(double angle){//Only pass in deltaTheta
    	
    	double deltaEncoder;
    	deltaEncoder = angle*(encoderUnitsPerRotation/360.0); 
    	
    	return (int)deltaEncoder;
    }
    
    public void drive(){
    	/*if(Math.abs(targetAngle-currentAngle) > 2)
    		driveMotor.set(speed*speedModifier);
    	else{
    		driveMotor.set(-speed*speedModifier);
    	}*/
    	SmartDashboard.putNumber("Speed: " + this.toString(), speed*speedModifier);
    	driveMotor.set(speed*speedModifier);//*directionalModifier
    }
	/*
	public void goToHome(){
		rotateMotor.enableLimitSwitch(true, false);
		rotateMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		while(!rotateMotor.isFwdLimitSwitchClosed()){
			rotateMotor.set(.2);
		}
		encoderAtHome = rotateMotor.getEncPosition();
		rotateMotor.enableLimitSwitch(false, false);
		
		rotateMotor.changeControlMode(CANTalon.ControlMode.Position);
		rotateMotor.set(encoderAtHome);
	}*/
    
	public void goToHome(){
		//rotateMotor.enableLimitSwitch(true, false);
		rotateMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		while(!rotateMotor.isFwdLimitSwitchClosed()){
			rotateMotor.set(.15);
		}
		encoderAtHome = rotateMotor.getEncPosition();
		SmartDashboard.putNumber("Encoder At Home: ", encoderAtHome);
		rotateMotor.set(0);
		rotateMotor.enableLimitSwitch(false, false);
		rotateMotor.changeControlMode(CANTalon.ControlMode.Position);
	}
	/*
	public void calibration(boolean saveValue){
    	SmartDashboard.putNumber("Encoder At Home: ", encoderAtHome);
    	SmartDashboard.putNumber("OffsetSavedValue", offsetFromZero);
    	SmartDashboard.putNumber("CurrentAngle: ", encoderUnitToAngle(rotateMotor.getEncPosition()));
		SmartDashboard.putNumber("Distance from Zero", encoderAtHome - rotateMotor.getEncPosition());
		
    	if (saveValue) {
    		offsetFromZero = (encoderAtHome - rotateMotor.getEncPosition());
    		SmartDashboard.putNumber("OffsetSavedValue", offsetFromZero);
    	}
    }*/
	
	public void goToZero(){
		goToHome();
		rotateMotor.setP(5);
		rotateMotor.setD(8);
		rotateMotor.set(encoderAtHome - homeToZero);
		/*
		while(rotateMotor.getEncPosition() - (encoderAtHome) <= 2){
			//wait
		}
		rotateMotor.changeControlMode(CANTalon.ControlMode.Position);*/
	}
	
	
    
    public void test(){
    	rotateMotor.set(encoderUnitsPerRotation*10);
    }

	
}
