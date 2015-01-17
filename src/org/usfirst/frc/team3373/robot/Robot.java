
package org.usfirst.frc.team3373.robot;


import com.kauailabs.nav6.frc.IMU; 
import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
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
    CANTalon talon;
    
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
    
    public Robot() {
        stick1 = new SuperJoystick(0);
        stick2 = new SuperJoystick(1);
        indexer = new Indexer();
        servo = new Servo(2);
        talon = new CANTalon (0);
        talon.changeControlMode(CANTalon.ControlMode.Speed);
        talon.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
        
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
            Timer.delay(0.005);		// wait for a motor update time
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
            
            if (imu.getYaw() > 0) {
            		servo.setAngle(imu.getYaw());
            } else if (imu.getYaw() < 0){
            	servo.setAngle(360+imu.getYaw());
            }
            
            SmartDashboard.putNumber("Servo Angle", servo.getAngle());
            SmartDashboard.putNumber("Pot", talon.get());
            Timer.delay(.01);
            talon.set(talon.get()/5);
    	}
    }
}
