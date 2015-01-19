package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.RobotDrive;

public class Indexer {
	
	RobotDrive indexer = new RobotDrive(9, 10);//using PWM ports 0 and 1 (using robotDrive to be able to use tank drive to control wheels)

	public void wheelControl(double leftY, double rightY){
		
		indexer.tankDrive(leftY, rightY);
		
	}
	
	
	
	
	
	
}
