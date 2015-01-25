package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.CANTalon;

public class Lifter {
	CANTalon leftActuator;
	CANTalon rightActuator;
	
	double pr = 10;
	double ir = 10;
	double dr = 10;
	
	double pl = 10;
	double il = 0;
	double dl = 10;
	
	//difference in position in pot value
	double offset = .05;
	
	/**
	 * Initializes Lifter class, feeds in motor values to control lifting motors
	 * @param leftActuatorID CANBus ID for left actuator
	 * @param rightActuatorID CANBus ID for right actuator
	 */
	public Lifter(int leftActuatorID, int rightActuatorID){
		leftActuator = new CANTalon(leftActuatorID);
		rightActuator = new CANTalon(rightActuatorID);
	
		//Sets PID
		rightActuator.setPID(pr, ir, dr);
		leftActuator.setPID(pl, il, dl);
		
		//Sets value to be fed to CANTalons (Position in this case, v -1 to 1 speed)
		rightActuator.changeControlMode(CANTalon.ControlMode.Position);
		leftActuator.changeControlMode(CANTalon.ControlMode.Position);
		
		//Sets main type of sensor available on the CANBus
		rightActuator.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		leftActuator.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
	}
	/**
	 * Given a target position, goes to the position
	 * @param targetPos position from 0-5 volts the lifter needs to go to
	 */
	public void goToPosition(double targetPos){
			rightActuator.set(targetPos);
			leftActuator.set(targetPos -offset);
	}
	
	
}
