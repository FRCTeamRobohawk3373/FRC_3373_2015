package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveWheel {

	Talon driveMotor;
	CANTalon rotateMotor;
	
	double rAngle;
	double speed;
	double targetAngle;
	double currentAngle;
	int encoderUnitsPerRotation = 1665;
	double speedModifier = 0.5;
	int encoderAtZero = 0;
	int offsetFromZero = 0;

	
	
	public SwerveWheel(int driveMotorChannel, int rotateMotorID, double p, double i, double d, double rotateAngle, int distanceFromZero){
		
		driveMotor = new Talon(driveMotorChannel);
		rotateMotor = new CANTalon(rotateMotorID);
		
		rotateMotor.setPID(p,i,d);
        rotateMotor.changeControlMode(CANTalon.ControlMode.Position);
        rotateMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rotateMotor.enableLimitSwitch(false, false);
        
        currentAngle = encoderUnitToAngle(rotateMotor.getEncPosition());
        targetAngle = rotateMotor.getEncPosition();
		rAngle = rotateAngle;
		offsetFromZero = distanceFromZero;
	}
	
	public double getDeltaTheta(){
		double deltaTheta = targetAngle - currentAngle;
		
		while ((deltaTheta < -90) || (deltaTheta > 90)){
			if(deltaTheta > 90){
				deltaTheta -= 180;
			}else if(deltaTheta < -90){
				deltaTheta += 180;
			}
		}
		if (deltaTheta >= 2 || deltaTheta <= -2){
			return deltaTheta;
		} else {
			return 0;
		}
		
	}
	
    public int encoderUnitToAngle(int encoderValue){
    	double angle = 0;
    	if (encoderValue >= 0){
    		angle = (encoderValue * (360.0/encoderUnitsPerRotation));
    		angle = angle % 360;
    	} else if (encoderValue < 0){
    		angle = 360 - (encoderValue * (360.0/encoderUnitsPerRotation));
    		angle = angle % 360;
    	}
    	return (int)angle;//(angle+2*(90-angle));
    }
    
    public void setSpeed(){
    	if(Math.abs(targetAngle-currentAngle) > 2)
    		driveMotor.set(speed*speedModifier);
    	else{
    		driveMotor.set(-speed*speedModifier);
    	}
    }
	
	public void goToHome(){
		rotateMotor.enableLimitSwitch(true, false);
		rotateMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		while(!rotateMotor.isFwdLimitSwitchClosed()){
			rotateMotor.set(.3);
		}
		rotateMotor.enableLimitSwitch(false, false);
		rotateMotor.changeControlMode(CANTalon.ControlMode.Position);
		encoderAtZero = rotateMotor.getEncPosition();
		SmartDashboard.putNumber("EncoderAtZero: ", encoderAtZero);
	}
	
	public void goToZero(){
		goToHome();
		rotateMotor.setP(3);
		rotateMotor.set(rotateMotor.getEncPosition() + offsetFromZero);
	}
	
	
    public void calibration(boolean saveValue){
    	SmartDashboard.putNumber("OffsetFromHome To Zero: ", encoderAtZero);
    	SmartDashboard.putNumber("OffsetSavedValue", offsetFromZero);
    	SmartDashboard.putNumber("CurrentAngle: ", encoderUnitToAngle(-rotateMotor.getEncPosition()));
    	
    	if (saveValue) {
    		offsetFromZero = (encoderAtZero - rotateMotor.getEncPosition());
    		SmartDashboard.putNumber("OffsetSavedValue", offsetFromZero);
    	}
    }

	
}
