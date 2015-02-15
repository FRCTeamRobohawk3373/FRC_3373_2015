package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	
	double potScalarR = 783;
	double potScalarL = 763;
	double casingLength = 12;
	//Right Actuator
	double maxPotValueR = 870;
	double minPotValueR = 142;
	double maxLengthR = 11.1875;//in inches
	double minLengthR = 1.75;//in inches
	//left Actuator
	double maxPotValueL = 839;
	double minPotValueL = 136;
	double maxLengthL = 11;//in inches
	double minLengthL = 1.8125;//in inches
	
	double diffBetweenPots = 0;
	
	double lifterTarget;
	
	//Have to use these as a work around to the PID controller class
	PIDOutputObject leftActPIDOutput = new PIDOutputObject();
	PIDOutputObject rightActPIDOutput = new PIDOutputObject();
	PIDOutputObject errorPIDOutput = new PIDOutputObject();
	
	PIDInputObject leftActPIDInput = new PIDInputObject();
	PIDInputObject rightActPIDInput = new PIDInputObject();
	PIDInputObject errorPIDInput = new PIDInputObject();
	
	PIDController leftActPID = new PIDController(.5, 0, 0, leftActPIDInput, leftActPIDOutput);
	PIDController rightActPID = new PIDController(.5, 0, 0, rightActPIDInput, rightActPIDOutput);
	PIDController errorPID = new PIDController(3, 0, 0, errorPIDInput, errorPIDOutput);
	
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
		
		lifterTarget = getLeftActuatorLength();
		
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
	
	public double heightToPosition(double targetHeight){
		double actuatorLength;
		cosTheta = (robotHeight - targetHeight)/armLength;
		actuatorLength = Math.sqrt(Math.pow(jointThetaH, 2) + Math.pow(jointThetaA, 2) - (2*jointThetaH*jointThetaA*cosTheta));
		
		return actuatorLengthToPot(actuatorLength, potScalarR);
	}
	
	/**
	 * given a target actuator length, returns a pot position
	 * @param actuatorTargetLength target length for to hit a certain lifter height
	 * @return encoder position (0-5)
	 */
	
	private double actuatorLengthToPot(double actuatorTargetLength, double potScalar){
		double potPosition;
		potPosition = (actuatorTargetLength-casingLength) * (potScalar/casingLength);
		
		return potPosition;
	}
	
	private double potValueToActuatorLength(double potValue, double potScalar){
	
		double actLength;
		actLength = (potValue) * (casingLength/potScalar);
	
		return actLength;
	}
	
	/*
	 * Jamie's Code
	 */
	
	
	public double getLeftPotValue(){
		return leftActuator.getAnalogInPosition();

	}
	
	public double getRightPotValue(){
		return leftActuator.getAnalogInPosition() + diffBetweenPots;
	}
	
	public void initPIDControllers(){
		leftActPID.enable();
		rightActPID.enable();
		errorPID.enable();
		//set input range
		leftActPID.setInputRange(minLengthL, maxLengthL);
		rightActPID.setInputRange(minLengthR, maxLengthR);
		//set output range
		leftActPID.setOutputRange(-0.5, 0.5);
		rightActPID.setOutputRange(-0.5, 0.5);
		errorPID.setInputRange(0, 1);
		errorPID.setOutputRange(-0.1, 0.1);
		errorPID.setSetpoint(0);
		
		leftActPID.setPercentTolerance(5);
		rightActPID.setPercentTolerance(5);
		errorPID.setPercentTolerance(100);
		//update PIDs
		updatePIDControllers();
	}
	
	public void updatePIDControllers(){
		leftActPIDInput.setValue(getLeftActuatorLength());
		//SmartDashboard.putNumber("Current Length", leftActuator.getAnalogInPosition());
		//SmartDashboard.putNumber("CurrentPos", leftActuator.getAnalogInPosition());
		rightActPIDInput.setValue(getRightActuatorLength());
		errorPIDInput.setValue(getLeftActuatorLength() - getRightActuatorLength());
	}
	
	public double getLeftActuatorLength(){
		double slope = (maxLengthL - minLengthL)/(maxPotValueL - minPotValueL);
		double potValue = leftActuator.getAnalogInPosition();
		double length;
		length = slope * (potValue - maxPotValueL) + maxLengthL;
		System.out.println("Pot Value: "+ potValue);
		return length;
	}
	public double getRightActuatorLength(){
		double slope = (maxLengthR - minLengthR)/(maxPotValueR - minPotValueR);
		double potValue = rightActuator.getAnalogInPosition();
		double length;
		length = slope * (potValue - maxPotValueR) + maxLengthR;
		return length;
	}
	public double inchesToPotValueL(double length){
		double slope = (maxPotValueR - minPotValueR)/(maxLengthR - minLengthR);
		double correspondingPotValue;
		correspondingPotValue = slope * (length - maxLengthR) + maxPotValueL;
		return correspondingPotValue;
	}
	
	/*public void extendLeft(double target){
		if(Math.abs(target-leftActuator.getAnalogInPosition()) > 2){
			updatePIDControllers();
			leftActPID.setSetpoint(target);//inchesToPotValueL(target));
			SmartDashboard.putNumber("Target: ", target);//inchesToPotValueL(target));
			SmartDashboard.putNumber("Speed of Left Talon", leftActPIDOutput.getPIDValue());
			leftActuator.set(leftActPIDOutput.getPIDValue());
		}
	}*/
	
	public void changeTarget(double target){
		lifterTarget = target;
	}
	
	
	public void goToLength(){//in inches
		//if((Math.abs(lifterTarget - getLeftActuatorLength()) > .15) || (Math.abs(lifterTarget - getRightActuatorLength()) > .15)){
			updatePIDControllers();
			//set setpoint for Pid loops
			leftActPID.setSetpoint(lifterTarget);
			rightActPID.setSetpoint(lifterTarget);
		
			SmartDashboard.putNumber("Target: ", lifterTarget);
			System.out.println("CurrentLeft" + getLeftActuatorLength());
			
		
			SmartDashboard.putNumber("Left Current Length: ", getLeftActuatorLength());
			SmartDashboard.putNumber("Right Current Length: ", getRightActuatorLength());
		
			SmartDashboard.putNumber("PID Output Left: ", leftActPIDOutput.getPIDValue());
			SmartDashboard.putNumber("PID Output Right: ", rightActPIDOutput.getPIDValue());
			SmartDashboard.putNumber("PID Output Error: ", errorPIDOutput.getPIDValue());
		
			//set motors to desired speed
			leftActuator.set(leftActPIDOutput.getPIDValue());// + errorPIDOutput.getPIDValue());
			SmartDashboard.putNumber("Left Actuator Speed: ", leftActuator.get());
			rightActuator.set(rightActPIDOutput.getPIDValue());// - errorPIDOutput.getPIDValue());
			SmartDashboard.putNumber("Right Actuator Speed: ", rightActuator.get());//.getPIDValue() + errorPIDOutput.getPIDValue());
		//}

	}
}
