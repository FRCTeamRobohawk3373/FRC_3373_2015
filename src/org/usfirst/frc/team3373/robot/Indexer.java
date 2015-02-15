package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer {
	
	private RobotDrive indexer;
	private CANTalon armMotor;
	private AnalogInput pot;
	private double max;
	private double min;
	private double maxOutput = 0.4;
	private double output = maxOutput;
	private double current;
	private double maxCurrent = 0.9;
	private double minCurrent = 0.7;
	
	/**
	 * Initializes an indexer object. 
	 * @param leftWheelChannel The channel where the left wheel is plugged in. 
	 * @param rightWheelChannel The channel where the right wheel is plugged in.
	 * @param armMotorChannel
	 * @param potAnalogChannel
	 */

	public Indexer(int leftWheelChannel , int rightWheelChannel, int armMotorID, int potAnalogChannel){
		indexer = new RobotDrive(leftWheelChannel, rightWheelChannel);
		
		armMotor = new CANTalon(armMotorID);
		pot = new AnalogInput(potAnalogChannel);
		
		armMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		armMotor.enableBrakeMode(true);
		armMotor.enableLimitSwitch(false, false);
	}
	
	public void wheelControl(double leftY, double rightY){
		indexer.tankDrive(leftY, rightY);
	}
	
	public void controlArms(double LX){
		if(current > maxCurrent){
			output -= 0.05;
		} else if(current < minCurrent){
			output += 0.05;
		}
		
		if(output > maxOutput){
			output = maxOutput;
		} else if(output < 0){
			output = 0;
		}
		
		if(LX > 0.1){
			armMotor.set(-output);
		} else if(LX < 0.1) {
			armMotor.set(output);
		} else{
			armMotor.set(0);
		}
	}
	
	public boolean isHolding(){
		if(output < (maxOutput)){
			return true;
		} else {
			return false;
		}
	}
	

	/*
	public void controlMotors(double LX, double RX){
		
		leftCurrent = leftArmMotor.getOutputCurrent();
		rightCurrent = rightArmMotor.getOutputCurrent();
		
		//check current on left
		if(leftCurrent > maxCurrent){
			leftOutput -= 0.05;
		} else if(leftCurrent < minCurrent){
			leftOutput += 0.05;
		}
		if(leftOutput > maxOutput){
			leftOutput = maxOutput;
		} else if(leftOutput < 0){
			leftOutput = 0;
		}
		
		//check current on right
		if(rightCurrent > maxCurrent){
			rightOutput -= 0.05;
		} else if(rightCurrent < minCurrent){
			rightOutput += 0.05;
		}
		if(rightOutput > maxOutput){
			rightOutput = maxOutput;
		} else if(rightOutput < 0){
			rightOutput = 0;
		}
		
		SmartDashboard.putNumber("Left Current: ", leftCurrent);
		SmartDashboard.putNumber("Right Current: ", rightCurrent);

		SmartDashboard.putNumber("Left Output ", leftOutput);
		SmartDashboard.putNumber("Right Output ", rightOutput);
		
		SmartDashboard.putNumber("Left Axis: ", LX);
		SmartDashboard.putNumber("Right Axis: ", RX);
		
		if(LX > 0.1){
			leftArmMotor.set(leftOutput);
		} else if(LX < -0.1){
			leftArmMotor.set(-leftOutput);
		} else{
			leftArmMotor.set(0);
		}
		
		if(RX > 0.1){
			rightArmMotor.set(rightOutput);
		} else if(RX < -0.1){
			rightArmMotor.set(-rightOutput);
		} else{
			rightArmMotor.set(0);
		}
		
		
		if(LX > 0.1){
			leftArmMotor.set(leftOutput);
		} else if(LX < 0.1){
			leftArmMotor.set(-leftOutput);
		} else {
			leftArmMotor.set(0);
		}
		
		if(RX > 0.1){
			rightArmMotor.set(rightOutput);
		} else if(RX < 0.1){
			rightArmMotor.set(-rightOutput);
		} else {
			rightArmMotor.set(0);
		}
	}*/

}