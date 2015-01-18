package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;



public class SwerveControl  {
	
	public double calculateTargetTheta(int targetAngle, int currentAngle){
		double angleOne = Math.abs(targetAngle - currentAngle);
		double angleTwo = Math.abs(targetAngle - currentAngle - 180);
		
		if (angleOne <= angleTwo){
			return angleOne;
		} else {
			return -angleTwo;
		
		}
	}
	
	public double getAngle(AnalogInput encoder){
		double angle = 0;
		return angle;
	}
}

