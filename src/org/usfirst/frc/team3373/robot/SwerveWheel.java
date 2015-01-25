package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Talon;

public class SwerveWheel {

	Talon driveMotor;
	CANTalon rotateMotor;
	
	double rAngle;
	double speed;
	double targetAngle;
	double currentAngle;
	int encoderUnitsPerRotation = 1665;

	
	
	public SwerveWheel(int driveMotorChannel, int rotateMotorID, double p, double i, double d, double rotateAngle){
		
		driveMotor = new Talon(driveMotorChannel);
		rotateMotor = new CANTalon(rotateMotorID);
		
		rotateMotor.setPID(p,i,d);
        rotateMotor.changeControlMode(CANTalon.ControlMode.Position);
        rotateMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        
        currentAngle = encoderUnitToAngle(rotateMotor.getEncPosition());
        targetAngle = rotateMotor.getEncPosition();
		rAngle = rotateAngle;
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
		
		return deltaTheta;
		
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
    	return (int)angle;
    }
	
	
}
