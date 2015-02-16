
package org.usfirst.frc.team3373.robot;


import com.kauailabs.nav6.frc.IMU; 
import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot extends SampleRobot {
    //Controllers
	SuperJoystick driver;
    SuperJoystick shooter;
    //Robot Subsystems
    Indexer indexer;
    SwerveControl swerve;
    Lifter lifter;
    
    //Controllers
    DigitalInput ones;
    DigitalInput twos;
    DigitalInput fours;
    DigitalInput eights;
    
    
    SerialPort serial_port;
    //IMU imu;  // Alternatively, use IMUAdvanced for advanced features
    IMUAdvanced imu;
    
    //Joystick Axes
    int LX = 0;
    int LY = 1;
    int Ltrigger = 2;
    int Rtrigger = 3;
    int RX = 4;
    int RY = 5;
    
    boolean first_iteration;
    

    /***************************
     * Robot Talon Identifier  *
     *		F                  *
     * 0 ------ 1              *
     * |        |              *
     * |        |              *
     * 2--------3              *
     ***************************/
    int frontLeftRotate = 0;
    int frontRightRotate = 1;
    int backLeftRotate = 2;
    int backRightRotate = 3;
    
    int frontLeftDrive = 0;
    int frontRightDrive = 1;
    int backLeftDrive = 2;
    int backRightDrive = 3;
    
    double robotWidth = 21;
    double robotLength = 29;
    
    boolean haveRun;
    
    
    public Robot() {
    	//Initialize controllers
        driver = new SuperJoystick(0);
        shooter = new SuperJoystick(1);
        //Initialize robot sub-systems
        lifter = new Lifter(4, 5);
        indexer = new Indexer(8, 9, 6);
        swerve = new SwerveControl(frontLeftDrive, frontLeftRotate, frontRightDrive, 
        		frontRightRotate, backLeftDrive, backLeftRotate, backRightDrive, 
        	    backRightRotate, robotWidth, robotLength);
        
        
        //LimitSwitches for Auto selector
        ones = new DigitalInput(6);
        twos = new DigitalInput(7);
        fours = new DigitalInput(8);
        eights = new DigitalInput(9);
        
        
        
        haveRun = false;

        
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
        first_iteration = true;
    }

    /**
     * Drive left & right motors for 2 seconds then stop
     */
    public void autonomous() {
    	/*
    	int index = 17;//for testing purposes
    	if(ones.get()){
    		index += 1;
    	}
    	if(twos.get()){
    		index += 2;
    	}
    	if(fours.get()){
    		index += 4;
    	}
    	if(eights.get()){
    		index += 8;
    	}
    	System.out.println(index);
    	switch(index){
    		case 0:
    			break;
    		case 1:
    			break;
    		case 2:
    			break;
    		case 3:
    			break;
    		case 4:
    			break;
    		case 5:
    			break;
    		case 6:
    			break;
    		case 7:
    			break;
    		case 8:
    			break;
    		case 9:
    			break;
    		case 10:
    			break;
    		case 11:
    			break;
    		case 12:
    			break;
    		case 13:
    			break;
    		case 14:
    			break;
    		case 15:
    			swerve.wheelsToZero();
    			break;
    		default:
    			swerve.relativeRotateRobot(60);
    	}*/
    	
    	
    	/*swerve.relativeMoveRobot(270, 0.3, 2);
    	swerve.relativeMoveRobot(0, 0.3, 2);
    	swerve.relativeMoveRobot(90, 0.3, 2);
    	swerve.relativeMoveRobot(180, 0.3, 2);
    	
    	double angle = 0;
    			
    	while(angle < 360){
        	swerve.relativeMoveRobot(angle, 0.3, 0.2);
        	angle += 10;
    	}*/
    	/*
    	try{
    		Thread.sleep(4000);
    		} catch(Exception e){
    		
    	}
    	swerve.relativeRotateRobot(-10);*/
    	
    }

    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
        
    	while (isOperatorControl() && isEnabled()) {
            
    		/****************************
    		 * Driver Specific Controls *
    		 ****************************/
    		
    		//Driving
    		swerve.move(driver.getRawAxis(LY), driver.getRawAxis(LX), driver.getRawAxis(RX));
    		if(driver.isLBHeld()){
    			//turbo mode
    		} else if(driver.isRBHeld()){
    			//sniper mode
    		} else{
    			//default speed
    		}
    		
    		//Switching Driving modes
    		if(driver.isRStickHeld()){
    			//Robot Centric mode
    		} else if(driver.isLStickPushed()){
    			//Field Centric mode
    		} else if(driver.getRawAxis(Rtrigger) > 0.2){
    			//Object Centric
    		} else if(driver.getRawAxis(Ltrigger) > 0.2){
    			//Hook/Load Centric
    		}
    		
    		/*****************************
    		 * Shooter Specific Controls *
    		 *****************************/
    		
    		//Index and arm control
    		indexer.wheelControl(shooter.getRawAxis(LY), shooter.getRawAxis(RY));
    		indexer.controlArms(shooter.getRawAxis(LX));//TODO: need to add control to RX also
    		
    		
    		//lifter control
    		if(shooter.isLBPushed()){
    			//up target by one tote
    		} else if(shooter.isRBPushed()){
    			//down target by one tote
    		}
    		//Manual lifter control
    		if(shooter.getRawAxis(Ltrigger) > 0.3){
    			//raise lifter
    		} else if(shooter.getRawAxis(Rtrigger) > 0.3){
    			//lower lifter
    		}
    		
    		/*******************
    		 * Shared Controls *
    		 *******************/
    		
    		if(driver.isAPushed() || shooter.isAPushed()){
    			//Lower lifter to the ground
    		} else if(driver.isBPushed() || shooter.isBPushed()){
    			//move lifter to transport height ~4 in off ground
    		} else if(driver.isXPushed() || shooter.isXPushed()){
    			//move lifter to can-pickup height
    		} else if(driver.isYPushed() || shooter.isYPushed()){
    			//unhook stack
    		}
    		//flip tote and right can
    		
    		if(driver.isStartPushed() || shooter.isStartPushed() || shooter.isRStickPushed()){
    			//right can
    		} else if(driver.isBackPushed() || shooter.isBackPushed() || shooter.isLStickPushed()){
    			//flip tote
    		}
    		
    		
    		
    		Timer.delay(0.005);
    		driver.clearButtons();
    		shooter.clearButtons();
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    	while (isTest() && isEnabled()){
    		//indexer.wheelControl(stick1.getRawAxis(LY), stick1.getRawAxis(RY));
    		//System.out.println("POV" + stick1.getPOV());
    		/*
    		if (stick1.isAHeld()){
    			swerve.FRWheel.targetAngle += 5;
    		} else if (stick1.isBHeld()){
    			swerve.FLWheel.targetAngle -= 5;
    		}
    		
				
    		double encoderFR = swerve.FRWheel.rotateMotor.getEncPosition();
    		*/
    		
            boolean is_calibrating = imu.isCalibrating();
            if ( first_iteration && !is_calibrating ) {
                Timer.delay( 0.3 );
                imu.zeroYaw();
                first_iteration = false;
            }
            
            //swerve.test();

            //indexer.controlMotors(stick1.getRawAxis(LX), stick1.getRawAxis(RX));
            
     
            
            SmartDashboard.putBoolean("Ones: ", ones.get());
            SmartDashboard.putBoolean("Twos: ", twos.get());
            SmartDashboard.putBoolean("Fours: ", fours.get());
            SmartDashboard.putBoolean("Eights: ", eights.get());
            
    		/*
            SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
            SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
            SmartDashboard.putNumber(   "IMU_Yaw",              imu.getYaw());
            SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
            SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
            SmartDashboard.putNumber(   "IMU_CompassHeading",   imu.getCompassHeading());
            SmartDashboard.putNumber(   "IMU_Update_Count",     imu.getUpdateCount());
            SmartDashboard.putNumber(   "IMU_Byte_Count",       imu.getByteCount());

            // If you are using the IMUAdvanced class, you can also access the following
            // additional functions, at the expense of some extra processing
            // that occurs on the CRio processor
            
            SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
            SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
            SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
            SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());*/
            
            /*
            if(haveRun != true){
            	swerve.FRWheel.goToHome();
            	haveRun = true;
            }
            
            
            swerve.FRWheel.calibration(stick1.isAPushed());*/
            
            //CENTRICITY Control aka switching robot modes
            
            /* SWERVE CODE
            
            if(stick1.isLStickPushed()){
            	swerve.switchToFieldCentric();
            }
            if(stick1.isRStickPushed()){
            	swerve.switchToObjectCentric();
            }
            if(stick1.isStartPushed()){
            	swerve.switchToRobotCentric();
            }
            
            
            swerve.changeOrientation(stick1.isYPushed(), stick1.isBPushed(), stick1.isAPushed(), stick1.isXPushed());
            swerve.move(-stick1.getRawAxis(LY), stick1.getRawAxis(LX), stick1.getRawAxis(RX));
            */
            
            /*SmartDashboard.putNumber("Back Left Current Encoder Reading", swerve.BLWheel.rotateMotor.getEncPosition());
            SmartDashboard.putNumber("Front Left Current Encoder Reading", swerve.FLWheel.rotateMotor.getEncPosition());
            SmartDashboard.putNumber("Back Right Current Encoder Reading", swerve.BRWheel.rotateMotor.getEncPosition());
            SmartDashboard.putNumber("Front Right Current Encoder Reading", swerve.FRWheel.rotateMotor.getEncPosition());
            
            SmartDashboard.putNumber("TargetAngleFL: ", swerve.FLWheel.targetAngle);
            SmartDashboard.putNumber("CurrentAngleFL", swerve.FLWheel.currentAngle);
            
            SmartDashboard.putNumber("TargetAngleFR: ", swerve.FRWheel.targetAngle);
            SmartDashboard.putNumber("CurrentAngleFR: ", swerve.FRWheel.currentAngle);
            
            SmartDashboard.putNumber("TargetAngleBR: ", swerve.BRWheel.targetAngle);
            SmartDashboard.putNumber("CurrentAngleBR: ", swerve.BRWheel.currentAngle);
            
            SmartDashboard.putNumber("TargetAngleBL: ", swerve.BLWheel.targetAngle);
            SmartDashboard.putNumber("CurrentAngleBL: ", swerve.BLWheel.currentAngle);
            
            SmartDashboard.putNumber("SpeedFL: ", swerve.FLWheel.speed);
            SmartDashboard.putNumber("SpeedFR: ", swerve.FRWheel.speed);
            SmartDashboard.putNumber("SpeedBL: ", swerve.BLWheel.speed);
            SmartDashboard.putNumber("SpeedBR: ", swerve.BRWheel.speed);*/
            
            //SmartDashboard.putBoolean("fwdLimit", actuator.isFwdLimitSwitchClosed());
            //SmartDashboard.putBoolean("RevLimit", actuator.isRevLimitSwitchClosed());
            Timer.delay(.01);

            /*if (stick1.isAPushed()){
            	twoTalon.set(20000);
            } else if (stick1.isBPushed()){
            	twoTalon.set(15000);
            }*/
            
            driver.clearButtons();
            shooter.clearButtons();
    	}
    }
    

}
