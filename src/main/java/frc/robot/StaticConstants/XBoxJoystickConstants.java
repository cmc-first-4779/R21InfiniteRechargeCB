/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.StaticConstants;

/**
 *  Settings that map out all of the buttons and sticks on the X-Box Controllers that we use.
 */
public class XBoxJoystickConstants {
    
//***************** JOYSTICK USB PORTS *******************************************/
	//Map out the Joystick #'s in the DriverStation USB Ports
	public static final int DRIVERSTICK_USB_PORT = 0;  // Driver Joystick - USB Port #0
	public static final int OPERSTICK_USB_PORT = 1;  // Operator Joystick - USB Port #1
  
  //***************** JOYSTICK BUTTONS *******************************************/
	//These are the buttons on the Joysticks as recognized by the Drivers Station. You can call in other subsystems.
	public static final int A_BUTTON = 1;  //  A button on X-Box Joystick is Button #1
	public static final int B_BUTTON = 2;  //  B button on X-Box Joystick is Button #2
	public static final int X_BUTTON = 3;  //  X button on X-Box Joystick is Button #3
	public static final int Y_BUTTON = 4;  //  Y button on X-Box Joystick is Button #4
	public static final int LEFT_BUMPER_BUTTON = 5;  // Left Bumper button on X-Box Joystick is Button #5
	public static final int RIGHT_BUMPER_BUTTON = 6; // Right Bumper button on X-Box Joystick is Button #6
	public static final int BACK_BUTTON = 7; // Back button on X-Box Joystick is Button #7
	public static final int START_BUTTON = 8; // Start button on X-Box Joystick is Button #8
	public static final int LEFT_STICK_BUTTON = 9; //  Button press by pressind down on Left Stick
	public static final int RIGHT_STICK_BUTTON = 10; // Button press by pressign down on Right Stick
    //  Joysticks and Triggers
	public static final int LEFT_STICK_X_AXIS = 0;  // X-axis on X-Box Joystick is axis #0
	public static final int LEFT_STICK_Y_AXIS = 1;  // Y-axis on X-Box Joystick is axis #1
	public static final int LEFT_TRIGGER = 2;  // Left Trigger on X-Box Joystick is axis #2
    public static final int RIGHT_TRIGGER = 3;  // Right Trigger on X-Box Joystick is axis #3
    public static final int RIGHT_STICK_X_AXIS = 4;  //  Right Stick X-axis 
    public static final int RIGHT_STICK_Y_AXIS = 5;  //  Right Stick Y-axis
}
