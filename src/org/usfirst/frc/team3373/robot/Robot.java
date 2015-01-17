
package org.usfirst.frc.team3373.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

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
    
    //Joystick Axes
    int LX = 1;
    int LY = 2;
    int triggers = 3;
    int RX = 4;
    int RY = 5;
    int DP = 6;
    
    public Robot() {
        stick1 = new SuperJoystick(0);
        stick2 = new SuperJoystick(1);
        indexer = new Indexer();
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
    		indexer.indexControl(stick1.getRawAxis(LY), stick1.getRawAxis(RY));
    	}
    }
}
