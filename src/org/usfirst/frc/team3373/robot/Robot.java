
package org.usfirst.frc.team3373.robot;


import com.kauailabs.nav6.frc.IMU; 
import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.CANTalon;
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
        rotateTalon = new Talon(0);
        driveTalon = new CANTalon (0);
        //driveTalon.changeControlMode(CANTalon.ControlMode.Position);
        driveTalon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);

        
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
            		servo.setAngle(imu.getYaw()/2);
            } else if (imu.getYaw() < 0){
            	servo.setAngle(180+imu.getYaw()/2);
            }
            
            SmartDashboard.putNumber("Servo Angle", servo.getAngle());
            double talonPosition = driveTalon.getEncPosition();
            SmartDashboard.putNumber("Pot", talonPosition);
            if (stick1.isXHeld()){
            	driveTalon.set(.5);
            } else if (stick1.isYHeld()){
            	driveTalon.set(-.5);
            } else {
            	driveTalon.set(0);
            }
            //rotateTalon.set(.3);
            Timer.delay(.01);
            //talon.set(talonPosition/1023);
            if (stick1.isAHeld()){
            	rotateTalon.set(.15);
            } else if (stick1.isBHeld()){
            	rotateTalon.set(-.15);
            } else {
            	rotateTalon.set(0);
            }
            SmartDashboard.putNumber("Angle: ", returnAngle(driveTalon.getEncPosition()));
            
            
    	}
    }
    
    public int returnAngle(int encoderValue){
    	System.out.println("Encoder Value: " + encoderValue);
    	int angle = 0;
    	if (encoderValue >= 0){
    		angle = (int)(encoderValue * (360.0/1652)) % 360;
    	} else if (encoderValue < 0){
    		angle = 360 - (int)(encoderValue * (360.0/1652)) % 360;
    	}
    	System.out.println(angle);
    	return angle;
    }
}
