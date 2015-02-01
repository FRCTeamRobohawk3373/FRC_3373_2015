package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveWheel {

	private Talon driveMotor;
	private CANTalon rotateMotor;
	
	private double rAngle;
	private double speed;
	private int targetAngle;
	private int encoderUnitsPerRotation = 1665;
	private double speedModifier = 0.3;
	private int encoderAtHome = 0;
	private int offsetFromZero = 0;
	private int directionalModifier = 1;

	
	
	public SwerveWheel(int driveMotorChannel, int rotateMotorID, double p, double i, double d, double rotateAngle, int distanceFromZero){
		
		driveMotor = new Talon(driveMotorChannel);
		rotateMotor = new CANTalon(rotateMotorID);
		
		rotateMotor.setPID(p,i,d);
        rotateMotor.changeControlMode(CANTalon.ControlMode.Position);
        rotateMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rotateMotor.enableLimitSwitch(false, false);
        
        targetAngle = encoderUnitToAngle(rotateMotor.getEncPosition());
		rAngle = rotateAngle;
		offsetFromZero = distanceFromZero;
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
			System.out.println(getTargetAngle());
			System.out.println(getCurrentAngle());
			System.out.println(deltaTheta);
		}
		
		System.out.println("New values: ");
		
		if (deltaTheta >= 2 || deltaTheta <= -2){
			return deltaTheta;
		} else {
			return 0;
		}
		
	}
	
	public void setTargetAngle(double angle){
		
		if(angle < 0){
			angle += 360;
		}else if(angle >=360){
			angle -= 360;
		}
		
		targetAngle = (int)angle;
	}
	
	public int getTargetAngle(){
		return targetAngle;
	}
	
	public int getCurrentAngle(){
		return encoderUnitToAngle(rotateMotor.getEncPosition());
	}
	
	public int getRAngle(){
		return (int)rAngle;
	}
	
	public void goToAngle(){
		rotateMotor.set(rotateMotor.getEncPosition() + angleToEncoderUnit(getDeltaTheta()));
	}
	
	public void setSpeed(double magnitude){
		speed = magnitude;
	}
	public double getSpeed(){
		return speed;
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
    	driveMotor.set(speed*speedModifier);//*directionalModifier
    }
	
	public void goToHome(){
		rotateMotor.enableLimitSwitch(true, false);
		rotateMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		while(!rotateMotor.isFwdLimitSwitchClosed()){
			rotateMotor.set(.5);
		}
		encoderAtHome = rotateMotor.getEncPosition();
		rotateMotor.enableLimitSwitch(false, false);
		rotateMotor.changeControlMode(CANTalon.ControlMode.Position);
		SmartDashboard.putNumber("EncoderAtZero: ", encoderAtHome);
	}
	
	public void goToZero(){
		goToHome();
		rotateMotor.setP(3);
		rotateMotor.set(encoderAtHome + offsetFromZero);
	}
	
	
    public void calibration(boolean saveValue){
    	SmartDashboard.putNumber("OffsetFromHome To Zero: ", encoderAtHome);
    	SmartDashboard.putNumber("OffsetSavedValue", offsetFromZero);
    	SmartDashboard.putNumber("CurrentAngle: ", encoderUnitToAngle(rotateMotor.getEncPosition()));
    	
    	if (saveValue) {
    		offsetFromZero = (encoderAtHome - rotateMotor.getEncPosition());
    		SmartDashboard.putNumber("OffsetSavedValue", offsetFromZero);
    	}
    }

	
}
