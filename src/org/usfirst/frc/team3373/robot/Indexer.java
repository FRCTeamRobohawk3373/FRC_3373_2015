package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;

public class Indexer {
	
	private RobotDrive indexer;
	private Talon armMotor;
	private PIDController armPID;
	private AnalogInput pot;
	
	/**
	 * Initializes an indexer object. 
	 * @param leftWheelChannel The channel where the left wheel is plugged in. 
	 * @param rightWheelChannel The channel where the right wheel is plugged in.
	 * @param armMotorChannel
	 * @param potAnalogChannel
	 */

	public Indexer(int leftWheelChannel , int rightWheelChannel, int armMotorChannel, int potAnalogChannel){
		indexer = new RobotDrive(leftWheelChannel, rightWheelChannel);
		armMotor = new Talon(armMotorChannel);
		pot = new AnalogInput(potAnalogChannel);
		
		armPID = new PIDController(3, 0, 5, pot, armMotor); 
	}
	
	public void wheelControl(double leftY, double rightY){
		indexer.tankDrive(leftY, rightY);
	}
	
	public void moveArmsToTargetPosition(double target){//target position in voltage
		armPID.setSetpoint(target);
	}

}