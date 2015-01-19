
package org.usfirst.frc.team3373.robot;


import com.kauailabs.nav6.frc.IMU; 
import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
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
    CANTalon driveTalon;
    Talon rotateTalon;
    PIDController pid;
    SwerveControl swerve;
    Deadband deadband;
    
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
    
    
    
    public Robot() {
        stick1 = new SuperJoystick(0);
        stick2 = new SuperJoystick(1);
        indexer = new Indexer();
        //servo = new Servo(2);
        swerve = new SwerveControl(0, 0, 1, 1, 2, 2, 3, 3);
        deadband = new Deadband();

        //driveTalon = new CANTalon (0);

        
        AnalogInput pot = new AnalogInput(0);
        
        //rotateTalon = new Talon(0);
        //driveTalon.setPID(p,i,d);
        //pid = new PIDController(proportionalConstant, derivativeConstant, integralConstant, pot, rotateTalon );
        
        
        
        /*driveTalon.changeControlMode(CANTalon.ControlMode.Position);
        driveTalon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        drivePos = driveTalon.getEncPosition();*/

        
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

    }

    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
        
    	while (isOperatorControl() && isEnabled()) {
            
        	//SmartDashboard.putNumber("PIDError", pid.getError());
        	if (stick1.isAPushed()){
        		drivePos += 500;
        	} else if (stick1.isBPushed()){
        		drivePos -= 500;
        	}
    		//driveTalon.set(drivePos);
        	
    		Timer.delay(0.005);		// wait for a motor update time
    		stick1.clearButtons();
    		stick2.clearButtons();
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    	while (isTest() && isEnabled()){
    		indexer.wheelControl(stick1.getRawAxis(LY), stick1.getRawAxis(RY));
    		//System.out.println("POV" + stick1.getPOV());
    		
            boolean is_calibrating = imu.isCalibrating();
            if ( first_iteration && !is_calibrating ) {
                Timer.delay( 0.3 );
                imu.zeroYaw();
                first_iteration = false;
            }
    		
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
            SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());
            
            
            swerve.move(deadband.zero(stick1.getRawAxis(LY), .1), deadband.zero(stick1.getRawAxis(LX), .1), deadband.zero(stick1.getRawAxis(RX), 0));

            Timer.delay(.01);

            stick1.clearButtons();
            stick2.clearButtons();
    	}
    }
    

}
