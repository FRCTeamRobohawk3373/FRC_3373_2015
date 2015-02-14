package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;

public class Lifter {
	CANTalon leftActuator;//currently ID 5
	CANTalon rightActuator;//currently ID 6
	
	double pr = 10;
	double ir = 10;
	double dr = 10;
	
	double pl = 10;
	double il = 0;
	double dl = 10;
	
	//height in inches
	double robotHeight = 60;
	
	//parallelogram lift length in inches
	double armLength = 60;
	
	double cosTheta;
	
	//difference in position in pot value
	double offset = .05;
	
	//distance between lift joint and actuator joint on height
	double jointThetaH = 12;
	
	//distance between lift joint and actuator joint on arm
	double jointThetaA = 24;
	
	double potScalar = 5;
	double casingLength = 12;
	
	double maxPotValueR;
	double minPotValueR;
	double maxPotValueL;
	double minPotValueL;
	
	double diffBetweenPots = 0;
	
	//Have to use these as a work around to the PID controller class
	PIDOutputObject leftActPIDOutput = new PIDOutputObject();
	PIDOutputObject rightActPIDOutput = new PIDOutputObject();
	PIDOutputObject errorPIDOutput = new PIDOutputObject();
	
	PIDInputObject leftActPIDInput = new PIDInputObject();
	PIDInputObject rightActPIDInput = new PIDInputObject();
	PIDInputObject errorPIDInput = new PIDInputObject();
	
	PIDController leftActPID = new PIDController(1, 0, 0, leftActPIDInput, leftActPIDOutput);
	PIDController rightActPID = new PIDController(1, 0, 0, rightActPIDInput, rightActPIDOutput);
	PIDController errorPID = new PIDController(1, 0, 0, errorPIDInput, errorPIDOutput);
	
	/**
	 * Initializes Lifter class, feeds in motor values to control lifting motors
	 * @param leftActuatorID CANBus ID for left actuator
	 * @param rightActuatorID CANBus ID for right actuator
	 */
	public Lifter(int leftActuatorID, int rightActuatorID){
		leftActuator = new CANTalon(leftActuatorID);
		rightActuator = new CANTalon(rightActuatorID);
		
		
		/*
		//Sets PID
		rightActuator.setPID(pr, ir, dr);
		leftActuator.setPID(pl, il, dl);
		
		//Sets value to be fed to CANTalons (Position in this case, v -1 to 1 speed)
		rightActuator.changeControlMode(CANTalon.ControlMode.Position);
		leftActuator.changeControlMode(CANTalon.ControlMode.Follower);
		
		//Sets main type of sensor available on the CANBus
		rightActuator.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		leftActuator.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		*/
		
		leftActuator.changeControlMode(CANTalon.ControlMode.PercentVbus);
		rightActuator.changeControlMode(CANTalon.ControlMode.PercentVbus);
		
		initPIDControllers();
	}
	
	/**
	 * Given a target position, goes to the position
	 * @param targetPos position from 0-5 volts the lifter needs to go to
	 */
	/*
	public void goToPosition(double targetPos){
			rightActuator.set(heightToPosition(targetPos));
			leftActuator.set(rightActuator.getDeviceID());
	}*/
	
	/**
	 * Converts target height to a position on the potentiometer 
	 * @param targetHeight target lifter height above the ground
	 * @return target pot position (0-5V)
	 */
	/*
	public double heightToPosition(double targetHeight){
		double actuatorLength;
		cosTheta = (robotHeight - targetHeight)/armLength;
		actuatorLength = Math.sqrt(Math.pow(jointThetaH, 2) + Math.pow(jointThetaA, 2) - (2*jointThetaH*jointThetaA*cosTheta));
		
		return actuatorLengthToPot(actuatorLength);
	}*/
	
	/**
	 * given a target actuator length, returns a pot position
	 * @param actuatorTargetLength target length for to hit a certain lifter height
	 * @return encoder position (0-5)
	 */
	/*
	private double actuatorLengthToPot(double actuatorTargetLength){
		double potPosition;
		potPosition = (actuatorTargetLength-casingLength) * (potScalar/casingLength);
		
		return potPosition;
	}
	*/
	
	/*
	 * Jamie's Code
	 */
	
	public void relateActuators(){
		double diffAtMax = maxPotValueR - maxPotValueL;
		double diffAtMin = minPotValueR - minPotValueL;
		double difference;
		
		if(diffAtMax == diffAtMin){
			difference = diffAtMax;
		}
	}
	
	public double getLeftPotValue(){
		return leftActuator.getAnalogInPosition();

	}
	
	public double getRightPotValue(){
		return leftActuator.getAnalogInPosition() + diffBetweenPots;
	}
	
	public void initPIDControllers(){
		//set input range
		leftActPID.setInputRange(minPotValueR, maxPotValueL);
		rightActPID.setInputRange(minPotValueR, maxPotValueR);
		//set output range
		leftActPID.setOutputRange(-0.5, 0.5);
		rightActPID.setOutputRange(-0.5, 0.5);
		errorPID.setOutputRange(-0.25, 0.25);
		errorPID.setSetpoint(0);
		//update PIDs
		updatePIDControllers();
	}
	
	public void updatePIDControllers(){
		leftActPIDInput.setValue(getLeftPotValue());
		rightActPIDInput.setValue(getRightPotValue());
		errorPIDInput.setValue(getLeftPotValue() - getRightPotValue());
	}
	
	
	public void goToPosition(double targetPosition){//in potentiometer units
		updatePIDControllers();
		//set setpoint for Pid loops
		leftActPID.setSetpoint(targetPosition);
		rightActPID.setSetpoint(targetPosition);
		//set motors to desired speed
		leftActuator.set(leftActPIDOutput.getPIDValue() - errorPIDOutput.getPIDValue());
		rightActuator.set(rightActPIDOutput.getPIDValue() + errorPIDOutput.getPIDValue());
	}
}
