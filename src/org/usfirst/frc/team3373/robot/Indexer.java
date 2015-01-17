package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.RobotDrive;

public class Indexer {
	
	RobotDrive indexer = new RobotDrive(0, 1);//using PWM ports 0 and 1 (using robotDrive to be able to use tank drive

	public void indexControl(double leftY, double rightY){
		
		indexer.tankDrive(leftY, rightY);
		
	}
	
	
	
	
	
	
}
