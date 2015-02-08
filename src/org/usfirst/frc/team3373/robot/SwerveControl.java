package org.usfirst.frc.team3373.robot;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveControl  {
	
	//must also use a wheel class
	
	Talon driveLFMotor;
	CANTalon rotateLFMotor;
	//Front Right Wheel
	Talon driveRFMotor;
	CANTalon rotateRFMotor;
	//Back Left Wheel
	Talon driveLBMotor;
	CANTalon rotateLBMotor;
	//Back Right Wheel
	Talon driveRBMotor;
	CANTalon rotateRBMotor;
	
	
	//NavX
	IMUAdvanced imu;
	SerialPort serial_port;
	
	int encoderUnitsPerRotation = 1660;//was 1665
	
    double p = 10; //100 is very close
    double i = 0;
    double d = 0;
    //double f = 0;
    //int izone = 100;

    double orientationOffset;
    
    //Used to switch between control modes
    boolean isRobotCentric = true;
    boolean isFieldCentric = false;
    boolean isObjectCentric = false;
    
    double radius = 40;
    
	SwerveWheel[] wheelArray;

	SwerveWheel FLWheel;
	SwerveWheel FRWheel;
	SwerveWheel BLWheel;
	SwerveWheel BRWheel;
	
	double angleToDiagonal;
	double robotLength;
	double robotWidth;
	
	/*give dimensions between the wheels both width and length, 
	 * width is the distance between left wheels and right wheels,
	 *  length is the distance between front wheels and back wheels*/
	
	public SwerveControl(int frontLeftDriveChannel, int frontLeftRotateID, int frontRightDriveChannel, 
			int frontRightRotateID, int backLeftDriveChannel, int backLeftRotateID, int backRightDriveChannel,
			int backRightRotateID, double width, double length){
		
		robotWidth = width;
		robotLength = length;
		
		angleToDiagonal = Math.toDegrees(Math.atan2(length, width));
		
		
		FLWheel = new SwerveWheel(frontLeftDriveChannel, frontLeftRotateID, p, i, d, (270 - angleToDiagonal), 687,1);
		FRWheel = new SwerveWheel(frontRightDriveChannel, frontRightRotateID, p, i, d, (angleToDiagonal + 90), 687,0);
		BLWheel = new SwerveWheel(backLeftDriveChannel, backLeftRotateID, p, i, d, (angleToDiagonal + 270), 687,2);
		BRWheel = new SwerveWheel(backRightDriveChannel, backRightRotateID, p, i, d, (90 - angleToDiagonal), 687,3);
		
		/*
		FLWheel = new SwerveWheel(frontLeftDriveChannel, frontLeftRotateID, p, i, d, (180 - angleToDiagonal), 0);
		FRWheel = new SwerveWheel(frontRightDriveChannel, frontRightRotateID, p, i, d, (angleToDiagonal), 0);
		BLWheel = new SwerveWheel(backLeftDriveChannel, backLeftRotateID, p, i, d, (angleToDiagonal + 180), 0);
		BRWheel = new SwerveWheel(backRightDriveChannel, backRightRotateID, p, i, d, (0 - angleToDiagonal), 0);
		*/
		
		
		wheelArray = new SwerveWheel[]{FLWheel, FRWheel, BLWheel, BRWheel};
		//wheelArray = new SwerveWheel[]{FRWheel};
		
        try {
        	serial_port = new SerialPort(57600,SerialPort.Port.kMXP);
    		
    		// You can add a second parameter to modify the 
    		// update rate (in hz) from 4 to 100.  The default is 100.
    		// If you need to minimize CPU load, you can set it to a
    		// lower value, as shown here, depending upon your needs.
    		
    		// You can also use the IMUAdvanced class for advanced
    		// features.
    		
    		byte update_rate_hz = 50;
    		//imu = new IMU(serial_port,update_rate_hz);
    		imu = new IMUAdvanced(serial_port,update_rate_hz);
        	} catch( Exception ex ) {
        		
        	}
		
		
		
	}
	
	double deltaTheta;
	
	double proprtionalConstant = .01;
	double derivativeConstant = 50;
	double integralConstant = 0;
	
	
	
	/*
    public int encoderUnitToAngle(int encoderValue){
    	
    	double angle = 0;
    	if (encoderValue >= 0){
    		angle = (encoderValue * (360.0/encoderUnitsPerRotation));
    		angle = angle % 360;
    	} else if (encoderValue < 0){
    		angle = (encoderValue * (360.0/encoderUnitsPerRotation));
    		angle = angle % 360;
    		angle += 360;
    	}
    	return (int)angle;//(angle+2*(90-angle));
    }
    */
	
    public int angleToEncoderUnit(double angle){//Only pass in deltaTheta
    	
    	double deltaEncoder;
    	deltaEncoder = angle*(encoderUnitsPerRotation/360.0); 
    	
    	return (int)deltaEncoder;
    }
    
    public void move(double LY, double LX, double RX){
    	if(isFieldCentric || isRobotCentric){
    		calculateSwerveControl(LY, LX, RX);
    	} else {
    		calculateObjectControl(RX);
    	}
    }
    
    
    public void calculateSwerveControl(double LY, double LX, double RX){
    	double translationalXComponent = LX;
    	double translationalYComponent = LY;
    	double translationalMagnitude;
    	double translationalAngle;
    	
    	double rAxis = RX;
    	double rotateXComponent;
    	double rotateYComponent;
    	double fastestSpeed = 0;
    	
    	if(Math.abs(LX) < 0.1){
    		translationalXComponent = 0;
    	}
    	
    	if(Math.abs(LY) < 0.1){
    		translationalYComponent = 0;
    	}
    	
    	if(Math.abs(RX) < 0.1){
    		rAxis = 0;
    	}
    	
    	
    	if(isFieldCentric){
    		orientationOffset = imu.getYaw();
    	}
    	
    	double rotationMagnitude = Math.abs(rAxis);
    	
    	translationalMagnitude = Math.sqrt(Math.pow(translationalYComponent, 2) + Math.pow(translationalXComponent, 2));
    	translationalAngle = Math.toDegrees(Math.atan2(translationalYComponent, translationalXComponent));
    	
    	translationalAngle += orientationOffset;
    	if(translationalAngle >= 360){
    		translationalAngle -= 360;
    	} else if(translationalAngle < 0){
    		translationalAngle += 360;
    	}
    	
    	translationalYComponent = Math.sin(Math.toRadians(translationalAngle)) * translationalMagnitude;
    	translationalXComponent = Math.cos(Math.toRadians(translationalAngle)) * translationalMagnitude;
    	
    	
    	//math for rotation and setting final angles and magnitudes for each wheel
    	for (SwerveWheel wheel : wheelArray){
    		
    		rotateXComponent = Math.cos(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
    		rotateYComponent = Math.sin(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
    		
    		if(rAxis > 0){
    			rotateXComponent = -rotateXComponent;
    			rotateYComponent = -rotateYComponent;
    		}
    		
    		wheel.setSpeed(Math.sqrt(Math.pow(rotateXComponent + translationalXComponent, 2) + Math.pow((rotateYComponent + translationalYComponent), 2)));
    		wheel.setTargetAngle(Math.toDegrees(Math.atan2((rotateYComponent + translationalYComponent), (rotateXComponent + translationalXComponent))));
    		
    		if(wheel.getSpeed() > fastestSpeed){
    			fastestSpeed = wheel.getSpeed();
    		}
    		
    		//wheel.getDeltaTheta();
    	}
    	
    	if(fastestSpeed > 1){
    		for(SwerveWheel wheel : wheelArray){
        		wheel.setSpeed(wheel.getSpeed()/fastestSpeed);
        	}
    	}
    	
    	
    	
    	//double FRWheelTarget = FRWheel.rotateMotor.getEncPosition() + angleToEncoderUnit(FRWheel.getDeltaTheta());
    	
    	FRWheel.goToAngle();
    	FLWheel.goToAngle();
    	BRWheel.goToAngle();
    	BLWheel.goToAngle();
    	
    	FRWheel.drive();
    	FLWheel.drive();
    	BRWheel.drive();
    	BLWheel.drive();
    	
    	
    	/*
    	//.rotateMotor.set(FRWheel.rotateMotor.getEncPosition() + angleToEncoderUnit(FRWheel.getDeltaTheta()));
    	SmartDashboard.putNumber("FR Target Encoder Position", (FRWheel.getTargetAngle()));
    	SmartDashboard.putNumber("FR DeltaTheta: ", angleToEncoderUnit(FRWheel.getDeltaTheta()));
    	SmartDashboard.putNumber("FR Current Encoder", FRWheel.getCurrentAngle());
    	try{
    		//Thread.sleep(5000);
    	}catch (Exception ex){
    		
    	}
    	
    	/*
    	FLWheel.rotateMotor.set(FLWheel.rotateMotor.getEncPosition() + angleToEncoderUnit(FLWheel.getDeltaTheta()));
    	SmartDashboard.putNumber("FL Target Encoder Position", (FLWheel.rotateMotor.getEncPosition() + angleToEncoderUnit(FLWheel.getDeltaTheta())));
    	SmartDashboard.putNumber("FL DeltaTheta: ", FLWheel.getDeltaTheta());
    	BRWheel.rotateMotor.set(BRWheel.rotateMotor.getEncPosition() + angleToEncoderUnit(BRWheel.getDeltaTheta()));
    	SmartDashboard.putNumber("BR Target Encoder Position", (BRWheel.rotateMotor.getEncPosition() + angleToEncoderUnit(BRWheel.getDeltaTheta())));
    	SmartDashboard.putNumber("BR DeltaTheta: ", BRWheel.getDeltaTheta());
    	BLWheel.rotateMotor.set(BLWheel.rotateMotor.getEncPosition() + angleToEncoderUnit(BLWheel.getDeltaTheta()));
    	SmartDashboard.putNumber("BL Target Encoder Position", (BLWheel.rotateMotor.getEncPosition() + angleToEncoderUnit(BLWheel.getDeltaTheta())));
    	SmartDashboard.putNumber("BL DeltaTheta: ", BLWheel.getDeltaTheta());
    	*/
    	

    	
    	//FRWheel.driveMotor.set(FRWheel.speed);
    	//FLWheel.driveMotor.set(FRWheel.speed);
    	//BRWheel.driveMotor.set(FRWheel.speed);
    	//BLWheel.driveMotor.set(FRWheel.speed);
    	
    	
    	//SmartDashboard.putNumber("Current Angle", BLWheel.getCurrentAngle());
    	//SmartDashboard.putNumber("Delta Theta", BLWheel.getDeltaTheta());
    	//SmartDashboard.putNumber("Target Angle", BLWheel.getTargetAngle());
    
    }
    
    public void calculateObjectControl(double RX){
    	double distanceToFront = radius - robotLength/2;
    	double distanceToBack = radius + robotLength/2;
    	
    	System.out.println("WE MADE IT");
    	
    	FLWheel.setTargetAngle(180 - Math.toDegrees(Math.atan2(robotWidth/2, distanceToFront)));
    	FRWheel.setTargetAngle(180 + Math.toDegrees(Math.atan2(robotWidth/2, distanceToFront)));
    	BLWheel.setTargetAngle(180 - Math.toDegrees(Math.atan2(robotWidth/2, distanceToBack)));
    	BRWheel.setTargetAngle(180 + Math.toDegrees(Math.atan2(robotWidth/2, distanceToBack)));
    	
    	BLWheel.setSpeed(RX);
    	BRWheel.setSpeed(RX);
    	
    	double speedRatio = Math.sqrt(Math.pow((robotWidth/2), 2) + Math.pow(distanceToFront, 2)) / Math.sqrt(Math.pow((robotWidth/2), 2) + Math.pow(distanceToBack, 2));
    	
    	FLWheel.setSpeed(speedRatio * RX);
    	FRWheel.setSpeed(speedRatio * RX);
    	
    	FRWheel.goToAngle();
    	FLWheel.goToAngle();
    	BRWheel.goToAngle();
    	BLWheel.goToAngle();
    	
    	FRWheel.drive();
    	FLWheel.drive();
    	BRWheel.drive();
    	BLWheel.drive();
    	
    }
    
    public void changeRadius(){
    	
    }

    
    public void changeOrientation(boolean north, boolean east, boolean south, boolean west){
    	
    	//switch out of field centric
    	//set the robot front (N,E,S,W)
    	if(north){
    		isFieldCentric = false;
    		isObjectCentric = false;
    		orientationOffset = 0;
   		} else if(east){
   			isFieldCentric = false;
   			isObjectCentric = false;
   			orientationOffset = -90;
   		} else if(south){
   			isFieldCentric = false;
   			isObjectCentric = false;
    		orientationOffset = 180;
    	} else if(west){
    		isFieldCentric = false;
    		isObjectCentric = false;
    		orientationOffset = 90;
    	}
    	
    }
    public void switchToFieldCentric(){
		isObjectCentric = false;
		isRobotCentric = false;
		isFieldCentric = true;
    }
    public void switchToObjectCentric(){
		isObjectCentric = true;
		isFieldCentric = false;
		isRobotCentric = false;
    }
    
    public void switchToRobotCentric(){
		isObjectCentric = false;
		isFieldCentric = false;
		isRobotCentric = true;
    }
    
    /*
    public void switchDrivingMode(boolean LStick, boolean RStick){
    	if(LStick && !isFieldCentric){
    		isObjectCentric = false;
    		isRobotCentric = false;
    		isFieldCentric = true;
    	} else if(LStick && isFieldCentric){
    		isObjectCentric = false;
    		isFieldCentric = false;
    		isRobotCentric = true;
    	} else if(RStick && !isObjectCentric){
    		isObjectCentric = true;
    		isFieldCentric = false;
    		isRobotCentric = false;
    	} else if(RStick && isObjectCentric){
    		isObjectCentric = false;
    		isFieldCentric = false;
    		isRobotCentric = true;
    	} else{
    		isObjectCentric = false;
    		isFieldCentric = false;
    		isRobotCentric = true;    		
    	}
    }*/
    
    
    public void wheelsToHomePos(){
    	/*for (SwerveWheel wheel : wheelArray){
    		wheel.goToHome();
    	}*/
    	FRWheel.goToHome();
    	//FRWheel.goToHome();
    	//BRWheel.goToHome();
    	//BLWheel.goToHome();
    
    }
    public void wheelsToZero(){
    	FRWheel.goToZero();
    	/*for (SwerveWheel wheel : wheelArray){
    		wheel.goToZero();
    		//wheel.rotateMotor.setP(10);
    	}*/
    	
    }
    
    
    
    public void test(){
    	FRWheel.test();
    	FLWheel.test();
    	BLWheel.test();
    	BRWheel.test();
    }
    
    /*public double calculateTargetDeltaTheta(int targetAngle, int currentAngle){
		double deltaThetaOne; 
		double deltaThetaTwo; 
		double deltaThetaOne360;
		double deltaThetaTwo360;
		double deltaThetaOneTarget;
		double deltaThetaTwoTarget;
	
		if (currentAngle == 0){
			deltaThetaOneTarget = Math.abs(targetAngle - 360);
			deltaThetaTwoTarget = Math.abs(targetAngle - 540);
		} else {
			deltaThetaOneTarget = 1000;
			deltaThetaTwoTarget = 1000;
		}
	
		if (targetAngle == 0){
			deltaThetaOne360 = Math.abs(360-currentAngle);
			deltaThetaTwo360 = Math.abs(360 - currentAngle - 180);
		} else {
			deltaThetaOne360 = 1000;
			deltaThetaTwo360 = 1000;
		}


		deltaThetaOne = Math.abs(targetAngle - currentAngle);
		deltaThetaTwo = Math.abs(targetAngle - currentAngle - 180);
	
		SmartDashboard.putNumber("DeltaThetaOne: ", deltaThetaOne);
		SmartDashboard.putNumber("DeltaThetaTwo: ", deltaThetaTwo);
		SmartDashboard.putNumber("DeltaThetaOne360: ", deltaThetaOne360);
		SmartDashboard.putNumber("DeltaThetaTwo360: ", deltaThetaTwo360);
		SmartDashboard.putNumber("DeltaThetaOneTarget: ", deltaThetaOneTarget);
		SmartDashboard.putNumber("DeltaThetaTwoTarget: ", deltaThetaTwoTarget);
	
		if (deltaThetaOne <= 90){
			return -deltaThetaOne;//if we get here DeltaTheta must be <= 90
		} else if (deltaThetaTwo <= 90){
			return deltaThetaTwo;//if we get here DeltaTheta must be <= 90
		} else if (deltaThetaOne360 <= 90){
			return -deltaThetaOne360;
		} else if (deltaThetaTwo360 <= 90){
			return deltaThetaTwo360;
		} else if (deltaThetaOneTarget <= 90){
			return -deltaThetaOneTarget;
		} else if (deltaThetaTwoTarget <= 90){
			return deltaThetaTwoTarget;
		} else return 0;
	
	}*/
    
    
    
    /*public void move(double LY, double LX, double RX){
    	double radians;
    	double deltaTheta;
    	double magnitude;
    	
    	radians = Math.atan2(LY, LX);
    	targetTheta = (int) Math.toDegrees(radians);
    	int currentTheta = encoderUnitToAngle(rotateRBMotor.getEncPosition());
    	//targetTheta = 5000;
    	

    	
    	rotateRBMotor.set(rotateRBMotor.getEncPosition()-angleToEncoderUnit((calculateTargetDeltaTheta(targetTheta, currentTheta))));
    	SmartDashboard.putNumber("RotateLBMotor", rotateLBMotor.get());
    	SmartDashboard.putNumber("Current Theta", currentTheta);
    	SmartDashboard.putNumber("TargetTheta", calculateTargetDeltaTheta(targetTheta, currentTheta));
    	//rotateLFMotor.set(targetTheta);
    	//rotateLBMotor.set(targetTheta);
    	//rotateRFMotor.set(targetTheta);
    	//rotateRBMotor.set(targetTheta);
    	
    	//driveLFMotor.set(0.25);
    	//driveLBMotor.set(0.25);
    	//driveRFMotor.set(0.25);
    	//driveRBMotor.set(0.25);
    }	
    
    public void move(boolean a, boolean b, boolean x, boolean y){
    	double radians;
    	double deltaTheta;
    	double magnitude;
    	
    	
    	if (a){
    		targetTheta = 90;
    	} else if (b){
    		targetTheta = 360;
    	} else if (x){
    		targetTheta = 180;
    	} else if (y){
    		targetTheta = 270;
    	} 
    	int currentTheta = encoderUnitToAngle(rotateRBMotor.getEncPosition());
    	//targetTheta = 5000;
    	


    	rotateRBMotor.set(angleToEncoderUnit((calculateTargetDeltaTheta(targetTheta, currentTheta))));
    	SmartDashboard.putNumber("RotateLBMotor", rotateLBMotor.get());
    	SmartDashboard.putNumber("Current Theta", currentTheta);
    	SmartDashboard.putNumber("TargetDeltaTheta", calculateTargetDeltaTheta(targetTheta, currentTheta));
    	SmartDashboard.putNumber("TargetTheta", targetTheta);
    	SmartDashboard.putNumber("EncoderDelta", angleToEncoderUnit((calculateTargetDeltaTheta(targetTheta, currentTheta))));
    	SmartDashboard.putNumber("Encoder Position", rotateRBMotor.getEncPosition());
    }*/
    
    /*
	public void move(double LY, double LX, double RX){//input the target angle position for wheel, current position of wheel, Talon for the rotating motor, CANTalon for drive motor
		double radians;
		double deltaTheta;
		double magnitude;
		
		//double deltaThetaLF;
		//double deltaThetaLB;
		//double deltaThetaRF;
		//double deltaThetaRB;
		
		radians = Math.atan2(LY, LX);
		targetTheta = Math.toDegrees(radians);
		
		if(targetTheta < 0){
			targetTheta += 360; //Get a positive equivalent angle
		}
		
		magnitude = Math.sqrt(LX*LX + LY*LY);
		
		deltaTheta = calculateTargetDeltaTheta((int)targetTheta, encoderUnitToAngle(rotateLFMotor.getEncPosition()));

        
        rotateLFMotor.set(rotateLFMotor.getEncPosition() + angleToEncoderUnit((int)deltaTheta));
        //if (deltaTheta > 0){
        //	driveLFMotor.set(magnitude);
        //} else if (deltaTheta < 0){
        //	driveLFMotor.set(-magnitude);
        //}
		
		SmartDashboard.putNumber("Target Angle: ", (int)targetTheta);
		SmartDashboard.putNumber("Delta Theta: ", deltaTheta);
		SmartDashboard.putNumber("Target Change", rotateLFMotor.getEncPosition() + angleToEncoderUnit((int)deltaTheta));
        SmartDashboard.putNumber("Current Encoder", rotateLFMotor.getEncPosition());
        SmartDashboard.putNumber("Current Angle", encoderUnitToAngle(rotateLFMotor.getEncPosition()));
		
	}*/
	

	
   
}

