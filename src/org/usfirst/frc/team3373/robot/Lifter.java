package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lifter {
	CANTalon leftActuator;//currently ID 5
	CANTalon rightActuator;//currently ID 6
	
	double pr = 10;
	double ir = 10;
	double dr = 2;
	
	double pl = 10;
	double il = 0;
	double dl = 2;
	
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
	double maxPotValueR = 934.0;
	double minPotValueR = 143.0;
	double maxLengthR = 12.0;//in inches
	double minLengthR = 1.0 + (3.0/4.0);//in inches
	//left Actuator
	double maxPotValueL = 1015.0;
	double minPotValueL = 279.0;
	double maxLengthL = 11 + (5.0/8.0);//in inches
	double minLengthL = 2.0 + (1.0/8.0);//in inches
	
	double diffBetweenPots = 0;
	
	double lifterTarget;
	
	boolean isRunning = false;
	
	boolean hasAlreadyMoved = false;
	
	double speedConstant = .1;

	
	//Have to use these as a work around to the PID controller class
	PIDOutputObject leftActPIDOutput = new PIDOutputObject();
	PIDOutputObject rightActPIDOutput = new PIDOutputObject();
	PIDOutputObject errorPIDOutput = new PIDOutputObject();
	
	PIDInputObject leftActPIDInput = new PIDInputObject();
	PIDInputObject rightActPIDInput = new PIDInputObject();
	PIDInputObject errorPIDInput = new PIDInputObject();
	
	PIDController leftActPID = new PIDController(10, 0, 5, leftActPIDInput, leftActPIDOutput);
	PIDController rightActPID = new PIDController(10, 0, 5, rightActPIDInput, rightActPIDOutput);
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
		 * */
		 
		rightActuator.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		leftActuator.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		
		
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
		leftActPID.setOutputRange(-0.2, 0.2);
		rightActPID.setOutputRange(-0.2, 0.2);
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
		double slope  = (maxLengthL - minLengthL)/(maxPotValueL - minPotValueL);
		double potValue = leftActuator.getAnalogInRaw();
		double length;
		length = slope * (potValue - maxPotValueL) + maxLengthL;
		return length;
	}
	public double getRightActuatorLength(){
		double slope = (maxLengthR - minLengthR)/(maxPotValueR - minPotValueR);
		SmartDashboard.putNumber("Slope", slope);
		double potValue = rightActuator.getAnalogInRaw();
		SmartDashboard.putNumber("RightActuatorPot", potValue);
		double length;
		length = (slope * (potValue - maxPotValueR)) + maxLengthR;
		return length;
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
	
	/**
	 * Sets the target acuator length
	 * @param target target actuator length
	 */
	public void changeTarget(double target){
		if (target > maxLengthL){
			lifterTarget = maxLengthL;
		}  else if (target < minLengthL){
			lifterTarget = minLengthL;
		} else {
			lifterTarget = target;
		}
	}
	
	
	public void goToLength(){//in inches
		//if((Math.abs(lifterTarget - getLeftActuatorLength()) > .15) || (Math.abs(lifterTarget - getRightActuatorLength()) > .15)){
			updatePIDControllers();
			//set setpoint for Pid loops
			leftActPID.setSetpoint(lifterTarget);
			rightActPID.setSetpoint(lifterTarget);
		
			SmartDashboard.putNumber("Target: ", lifterTarget);
			
		
			SmartDashboard.putNumber("Left Current Length: ", getLeftActuatorLength());
			SmartDashboard.putNumber("Right Current Length: ", getRightActuatorLength());
		
			SmartDashboard.putNumber("PID Output Left: ", leftActPIDOutput.getPIDValue());
			SmartDashboard.putNumber("PID Output Right: ", rightActPIDOutput.getPIDValue());
			SmartDashboard.putNumber("PID Output Error: ", errorPIDOutput.getPIDValue());
		
			//set motors to desired speed
			System.out.println("Moving MotorL: " + leftActPIDOutput.getPIDValue());
			System.out.println("PotValueL" + leftActuator.getAnalogInPosition());
			
			System.out.println("Moving MotorR: " + rightActPIDOutput.getPIDValue());
			System.out.println("PotValuer" + rightActuator.getAnalogInPosition());
			
			leftActuator.set(leftActPIDOutput.getPIDValue());// + errorPIDOutput.getPIDValue());
			SmartDashboard.putNumber("Left Actuator Speed: ", leftActuator.get());
			rightActuator.set(rightActPIDOutput.getPIDValue());// - errorPIDOutput.getPIDValue());
			SmartDashboard.putNumber("Right Actuator Speed: ", rightActuator.get());//.getPIDValue() + errorPIDOutput.getPIDValue());
		//}
			//Does not allow the robot to break itself by throwing arm out of wack
			/*
			if ((leftActuator.isFwdLimitSwitchClosed() || rightActuator.isFwdLimitSwitchClosed()) && ((lifterTarget > getLeftActuatorLength()) || (lifterTarget > getRightActuatorLength()))){
				leftActuator.set(0);
				rightActuator.set(0);
			} else if ((leftActuator.isRevLimitSwitchClosed() || rightActuator.isRevLimitSwitchClosed()) && ((lifterTarget < getLeftActuatorLength()) || (lifterTarget < getRightActuatorLength()))){
				leftActuator.set(0);
				rightActuator.set(0);
			} else {

			}*/

	}
	/**
	 * Raises the lifterArm for manual control
	 */
	public void raise(){
		changeTarget(lifterTarget += .01);
	}
	/**
	 * Lowers the lifter arm for manual control
	 */
	public void lower(){
		changeTarget(lifterTarget -= .01);
	}
	
	public void absoluteRaise(){
		leftActuator.set(.4);
		rightActuator.set(.4);
	}
	
	public void absoluteLower(){
		leftActuator.set(-.4);
		rightActuator.set(-.4);
	}
	
	public void absoluteStop(){
		leftActuator.set(0);
		rightActuator.set(0);
	}
	
	public void moveLeft(int direction){
		leftActuator.set(.4 * direction);
	}
	
	public void moveRight(int direction){
		rightActuator.set(.4 * direction);
	}
	
	public void printPotValues(){
		
		SmartDashboard.putNumber("LeftPotRaw", leftActuator.getAnalogInRaw());
		SmartDashboard.putNumber("RightPotRaw", rightActuator.getAnalogInRaw());
		
		SmartDashboard.putBoolean("LeftFWD", leftActuator.isFwdLimitSwitchClosed());
		SmartDashboard.putBoolean("LeftREV", leftActuator.isRevLimitSwitchClosed());
		SmartDashboard.putBoolean("RightFWD", rightActuator.isFwdLimitSwitchClosed());
		SmartDashboard.putBoolean("RightREV", rightActuator.isRevLimitSwitchClosed());
		
		SmartDashboard.putNumber("Target", lifterTarget);
		
		SmartDashboard.putNumber("LeftHeight", getLeftActuatorLength());
		SmartDashboard.putNumber("RightHeight", getRightActuatorLength());
	}
	/**
	 * Always add to rightActuatorSpeed
	 * @return modifier to cause right actuator to catch up to left
	 */
	public double speedModifier(){
		double speedModifier;
		speedModifier = (getLeftActuatorLength() - getRightActuatorLength()) * speedConstant;
		return speedModifier;
	}
	
	public void threadedGoToPos(boolean isEnabled){
		Thread thread = new Thread(new Runnable(){
			public void run(){
				isRunning = true;
				System.out.println("In thread");
				/*while (Math.abs(targetPosition - getLeftActuatorLength()) > .1 ){
					if ((targetPosition > getLeftActuatorLength())){// && (Math.abs(getLeftActuatorLength() - getRightActuatorLength()) < .15)){
						leftActuator.set(.5);
						System.out.println("Going up");
					} else if ((targetPosition < getLeftActuatorLength())){// && (Math.abs(getLeftActuatorLength() - getRightActuatorLength()) < .15)){
						leftActuator.set(-.5);
						System.out.println ("Going down");
					} else if (targetPosition == getLeftActuatorLength()){
						leftActuator.set(0);
					}
					
					if (targetPosition > getRightActuatorLength()){
						rightActuator.set(.5);

					} else if (targetPosition < getRightActuatorLength()){
						rightActuator.set(-.5);
					} else if (targetPosition == getRightActuatorLength()){
						leftActuator.set(0);
					}
					System.out.println("In thread");
				
				}*/
				if (getRightActuatorLength() < lifterTarget){//getLeftActuatorLength() < lifterTarget || getRightActuatorLength() < lifterTarget) {
						double speed = 0.3;
						double leftSpeed = 0.0;
						double rightSpeed = 0.0;
						while ((lifterTarget - getRightActuatorLength()) > 0){//(lifterTarget - getLeftActuatorLength()) > 0 || (lifterTarget - getRightActuatorLength()) > 0){
							if ((lifterTarget - getLeftActuatorLength()) > 0){
								leftSpeed = speed;
							} else {
								leftSpeed = 0;
							}
							
							if ((lifterTarget - getRightActuatorLength()) > 0){
								rightSpeed = speed;
							} else {
								rightSpeed = 0;
							}
							double offset = getLeftActuatorLength() - getRightActuatorLength();
							
							SmartDashboard.putNumber("Offset", offset);

							double slope = .1;
							//rightSpeed += offset * slope;
							
							leftActuator.set(leftSpeed);
							rightActuator.set(rightSpeed);
							hasAlreadyMoved = true;
						}
				}
			
				
				if ((getRightActuatorLength() > lifterTarget) && !hasAlreadyMoved){//(getLeftActuatorLength() > lifterTarget || getRightActuatorLength() > lifterTarget) && !hasAlreadyMoved) {
					double speed = -0.3;
					double leftSpeed = 0.0;
					double rightSpeed = 0.0;
					while ((lifterTarget - getRightActuatorLength()) < 0){//(lifterTarget - getLeftActuatorLength()) < 0 || (lifterTarget - getRightActuatorLength()) < 0){
						if ((lifterTarget - getLeftActuatorLength()) < 0){
							leftSpeed = speed;
						} else {
							leftSpeed = 0;
						}
						
						if ((lifterTarget - getRightActuatorLength()) < 0){
							rightSpeed = speed;
						} else {
							rightSpeed = 0;
						}
						double offset = getLeftActuatorLength() - getRightActuatorLength();
						
						SmartDashboard.putNumber("Offset", offset);

						double slope = 1;
						//rightSpeed += offset * slope;
						
						leftActuator.set(leftSpeed);
						rightActuator.set(rightSpeed);
					}
			}
				/*if (!hasAlreadyMoved && ((getLeftActuatorLength() > lifterTarget || getRightActuatorLength() > lifterTarget))){
					while ((getLeftActuatorLength() != lifterTarget || getRightActuatorLength() != lifterTarget)){
						if (getLeftActuatorLength() > lifterTarget){
							if (speedModifier() < 0){
								leftActuator.set(-.3 - speedModifier());
							} else {
								leftActuator.set(-.3);
							}
							System.out.println("Ldown");
						} else {
							leftActuator.set(0);
						}
						
						if (getRightActuatorLength() > lifterTarget){
							if (speedModifier() > 0){
								rightActuator.set(-.3 + speedModifier());
							} else {
								rightActuator.set(-.3);
							}
							System.out.println ("Rdown");
						} else {
							rightActuator.set(0);
						}
						
						System.out.println(hasAlreadyMoved);
					}
				}*/
				
				
				leftActuator.set(0);
				rightActuator.set(0);
				hasAlreadyMoved = false;
				isRunning = false;
			}
		});
		
		if (!isRunning){
			thread.start();
		}
	}
}
