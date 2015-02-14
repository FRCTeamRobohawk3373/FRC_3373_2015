
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
    SuperJoystick stick1;
    SuperJoystick stick2;
    Indexer indexer;
    Servo servo;
    Talon rotateTalon;
    PIDController pid;
    SwerveControl swerve;
    Deadband deadband;
    DigitalInput ones;
    DigitalInput twos;
    DigitalInput fours;
    DigitalInput eights;
    

    //CANTalon actuator;
    
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
    
    double proportionalConstant = 0.5;
    double derivativeConstant = 0.5;
    double integralConstant = 0.5;
    
    double p = 35; //100 is very close
    double i = 0;
    double d = 0;
    double f = 0;
    int izone = 100;
    double ramprate = 36;
    int profile = 0;
    int drivePos = 1;
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
        stick1 = new SuperJoystick(0);
        stick2 = new SuperJoystick(1);
        indexer = new Indexer(9, 10, 8, 1);
        //servo = new Servo(2);
        swerve = new SwerveControl(frontLeftDrive, frontLeftRotate, frontRightDrive, 
        		frontRightRotate, backLeftDrive, backLeftRotate, backRightDrive, 
        		backRightRotate, robotWidth, robotLength);
        deadband = new Deadband();
        
        //LimitSwitches for Auto selector
        ones = new DigitalInput(6);
        twos = new DigitalInput(7);
        fours = new DigitalInput(8);
        eights = new DigitalInput(9);
        
        //actuator = new CANTalon(0);
        

        //driveTalon = new CANTalon (0);

        
        
        haveRun = false;
        
        //actuator.setPID(p,i,d);
        //pid = new PIDController(proportionalConstant, derivativeConstant, integralConstant, pot, rotateTalon );
        
        
        
        //twoTalon.changeControlMode(CANTalon.ControlMode.Position);
        //twoTalon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        //drivePos = driveTalon.getEncPosition();

        
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
    	
    	//swerve.relativeRotateRobot(90);
    	swerve.absoluteRotateRobot(35);
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
            
    		
    		Timer.delay(0.005);
    		stick1.clearButtons();
    		stick2.clearButtons();
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
    		swerve.FRWheel.targetAngle = 358;
    		double FRCurrentAngle = swerve.FRWheel.currentAngle = swerve.encoderUnitToAngle(-swerve.FRWheel.rotateMotor.getEncPosition());
    		
			double deltaFR = swerve.FRWheel.getDeltaTheta();
			double deltaFL = swerve.FLWheel.getDeltaTheta();
			double deltaBR = swerve.BRWheel.getDeltaTheta();
			double deltaBL = swerve.BLWheel.getDeltaTheta();
    			
    		double encoderFR = swerve.FRWheel.rotateMotor.getEncPosition();
    		*/
    		
            boolean is_calibrating = imu.isCalibrating();
            if ( first_iteration && !is_calibrating ) {
                Timer.delay( 0.3 );
                imu.zeroYaw();
                first_iteration = false;
            }
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
            

            
            SmartDashboard.putNumber("LY: ", -stick1.getRawAxis(LY));
            SmartDashboard.putNumber("LX: ", stick1.getRawAxis(LX));
            SmartDashboard.putNumber("R: ", stick1.getRawAxis(RX));
            
            
            
            
            
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
            
            stick1.clearButtons();
            stick2.clearButtons();
    	}
    }
    

}
