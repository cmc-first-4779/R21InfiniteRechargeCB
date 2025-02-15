/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.StaticConstants.BlingConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  /*********************************************************************************/
  /*********************************************************************************/
  /**
   * SETTINGS THAT DEFINE PCM, PWM, DIO PORTS AND CONTROL MODULES
   ***************/
  /***
   * (Do NOT Change these unless we have to rewire something on the Robot)
   *********/
  /*********************************************************************************/
  /*********************************************************************************/

  // ***************** CAN ADDRESSES *******************************************/
  // Any Victors that are slaved to Talons have a CAN address of 10 + the TALON
  // address that it is slaved to.
  public static final int CAN_ADDRESS_FRONT_LEFT_DRIVE = 1; // Front Left Drive Motor (FALCON)
  public static final int CAN_ADDRESS_REAR_LEFT_DRIVE = 11; // Rear Left Drive Motor (FALCON)
  public static final int CAN_ADDRESS_FRONT_RIGHT_DRIVE = 2; /// Front Right Drive Motor (FALCON)
  public static final int CAN_ADDRESS_REAR_RIGHT_DRIVE = 12; // Rear Right Drive Motor (FALCON)
  public static final int CAN_ADDRESS_SHOOTER_MASTER = 21; // Shooter Motor (FALCON)
  public static final int CAN_ADDRESS_SHOOTER_SLAVE = 22; // Shooter Motor Slave (FALCON)
  public static final int CAN_ADDRESS_INTAKE = 31; // SparkMax controller w/ Neo 550 motor
  public static final int CAN_ADDRESS_HOPPER = 41; // SparkMax controlller w/ Neo 550 motor

  // ***************** PWM PORTS ON THE ROBORIO
  // **********************************/
  public static final int PWM_PORT_ELEVATOR_MOTOR = 0; // CIM Motor
  public static final int PWM_PORT_TURRET_MOTOR = 1; // Spark controller w/ RedLine (775) Motor
  public static final int PWM_PORT_BLING = 2; // PWM for BLINKIN LED Driver (Looks like a SPARK, but not)
  // public static final int PWM_PORT_TURRET = 0; // PWM for Turret subsystem

  // ************** PCM (PNEUMATICS CONTROL MODULE) PORTS
  // *************************/
  public static final int PCM_PORT_INTAKE_EXTEND = 0; // Solenoid for extending the intake
  public static final int PCM_PORT_INTAKE_RETRACT = 1; // PCM for retracting the intake

  /**********************
   * GYRO SPI PORT
   **************************************************/
  public static final int SPI_PORT_AHRS = 0; // Using Nav-X GYRO THIS YEAR

  // Set a DEFAULT BLING mode here so that way we only have to change it in one
  // place..
  public static final double BLING_DEFAULT = BlingConstants.BLING_OCEAN_PALETTE;

  /*********************************************************************************/
  /*********************************************************************************/
  /*************
   * ** SETTINGS WE CAN CHANGE TO TUNE THE ROBOT
   ***********************/
  /*********************************************************************************/
  /*********************************************************************************/

  /******************************************************************************* */
  /**************** DRIVE TRAIN SETTINGS ***************************/
  /******************************************************************************* */
  public static final double DRIVERTRAIN_MM_WAIT_SECS = 1; // Let Motion Magic settle down
  public static final double DRIVETRAIN_JOYSTICK_Y_THROTTLE = 1; // Throttle down the drive with joystick
  public static final double DRIVETRAIN_JOYSTICK_X_THROTTLE = 0.7; // Throttle down the drive with joystick
  public static final double DRIVETRAIN_WHEEL_DIAMETER = 6.0;
  public static final double DRIVETRAIN_GEAR_REDUCTION_RATIO = 7.31;
  public static final double DRIVETRAIN_ENCODER_TICKS_PER_ROTATION = 2048; // 80 for CIM, 4096 Redline encoder, 8192 for
                                                                           // falcon
  public static final int DRIVETRAIN_STRAIGHT_COUNTER = 20; // We use this to get motion magic a chance to line up
  public static final int DRIVETRAIN_DIRECTION_FORWARD = 1;
  public static final int DRIVETRAIN_DIRECTION_REVERSE = -1;
  public static final double DRIVETRAIN_TURN_POWER = 0.6;
  public static final double DRIVETRAIN_TURN_KP = 0.003;
  public static final double DRIVETRAIN_STABILIZE_WAIT_TIME_SECS = 0.25;
  public static double DRIVETRAIN_POSITION_TOLERANCE = 150;
  public static final int  DRIVETRAIN_MM_VELOCITY = 10000;
  public static final int  DRIVETRAIN_MM_ACCELERATION = 6000;

  // **************** HOPPER SETTINGS ****************/
  public static final double HOPPER_MOTOR_FORWARD_SPEED = .7; // How fast to move conveyor belt forward
  public static final double HOPPER_MOTOR_BACKWARD_SPEED = .7; // How fast to move the conveyor belt forward

  // **************** INTAKE SETTINGS ****************/
  public static final double INTAKE_MOTOR_SPEED = 1.0; // How fast to spin the rollers to intake a ball
  public static final double EJECT_MOTOR_SPEED = 0.8; // How fast to eject a ball from the intake
  public static final double INTAKE_ROLLER_TIMER = 5; // How long roller will run after retracting intake
  public static final double INTAKE_ROLLER_DELAY_TIMER = 1; // How lon the rollers wait after intake extends

  // **************** ELEVATOR SETTINGS ****************/
  public static final double ELEVATOR_MOTOR_UP_SPEED = 0.9; // How fast to spin the wheels for bring power cells up to
                                                            // shooter
  public static final double ELEVATOR_MOTOR_DOWN_SPEED = 0.9; // How fast the elevator will move cells up

  // **************** TURRET SETTINGS ****************/
  public static final double TURRET_MOTOR_SPEED = 0.2; // How fast to spin the turret to line-up to target
  public static final double TURRET_MIN_AIM_SPEED = 0.15;
  public static final double TURRET_SPEED_P = 0.02;
  public static final double TURRET_AIMING_DEADBAND = .75;  //degrees
  public static final int TURRET_AIMING_COUNTER = 10;  // Ten times trying to dial in
  
  public static final int TURRET_ANALOG_ENCODER_PORT = 0; // The analog port number to plug the turret encoder into on
                                                          // RoboRIO
  public static final int TURRET_NUMBER_ENCODER_PULSES_PER_REVOLUTION = 4096; // This value will change
  public static final double TURRET_RIGHT_STOP_LOCATION_ENCODER_POSITION = 3.8; // Analog Encoder value at Right Stop
                                                                                // position (in volts)
  public static final double TURRET_LEFT_STOP_LOCATION_ENCODER_POSITION = 1.3; // Analog Encoder value at Left Stop
                                                                               // position (in volts)

  // **************** SHOOTER SETTINGS ****************/
  public static final double SHOOTER_REGAIN_SPEED_TIMER = 0.5;
  public static final double SHOOTER_EMPTY_MAGAZINE_TIMER = 0.5;

  // AUTON CONSTANTS
  public static final double LIMELIGHT_SEEK_TIMEOUT_SECS = 2.0;

  /************************************************************************/
  /*************** LIMELIGHT / VISION SETTINGS *****************************/
  /********** These variables can change to be tuned..******************** */
  /********** Non-tunable variables are in LimelightConstants.java ***** */
  /******************************** =2 ****************************************/
  public static final double LIMELIGHT_PIPELINE_PORT_CLOSE = 0; // Use the Upper Power Port Pipeline
  public static final double LIMELIGHT_PIPELINE_PORT_FAR = 1;  // from > 14 ft away
  public static final double LIMELIGHT_PIPELINE_POWERCELL = 0; //  to gobble up powercells.   Seperate limelight
  public static final double LIMELIGHT_AIMING_DEADBAND = 1; // was 1.75 // How close do we have to be right or left of
                                                            // the target
  public static final double LIMELIGHT_AIMING_DISTANCE_TOLERANCE = 2; // Tolerance of degrees we want to be on target
  public static final double LIMELIGHT_AIMING_kpAim = 0.0040;
  public static final double LIMELIGHT_AIMING_kpDist = 0.05;
  public static final double LIMELIGHT_AIMING_AIM_MIN_CMD = 0.113; // adjust this for turn speed once we found a target
  public static final double LIMELIGHT_AIMING_MOVE_MIN_CMD = 0.26;
  public static final int LIMELIGHT_AIMING_COUNTER = 10; // Number of times we want to aim before taking a shot

  // LIMELIGHT SEEK MODE - How much power do we give the motors when it is turning
  // to scan and driving...
  public static final double LIMELIGHT_SEEK_TURN_DT_POWER = 0.45; // Power when we are turning when we CAN'T SEE THE
                                                                  // target
  public static final double LIMELIGHT_SEEK_DRIVE_DT_POWER = 0.7; // Power when we are driving toward the target
  public static final double LIMELIGHT_SEEK_TURN_TURRET_POWER = 0.4; // Power when we are using the turret to seek

  // public static double LIMELIGHT_SEEK_AREA = 0.9;

  // //Tolerance of degrees we want to be on target

  public static final double LIMELIGHT_SKEW_CLOCKWISE_MAX = 62.5; // degrees
  public static final double LIMELIGHT_SKEW_CLOCKWISE_MIN = -90.0; // degrees
  public static final double LIMELIGHT_SKEW_COUNTERCLOCKWISE_MAX = -0.01; // degrees
  public static final double LIMELIGHT_SKEW_COUNTERCLOCKWISE_MIN = 32.5; // degrees

  // *****************************************************************************
  // */
  // SHOOTER VELOCITY TALON PID CONSTANTS
  public static final int SHOOTER_TARGET_VELOCITY_UNITS_100MILLISECONDS_HIGH = 16300; // placeholder
  public static final int SHOOTER_TARGET_VELOCITY_UNITS_100MILLISECONDS_LOW = 8000; // placeholder
  public static final int SHOOTER_TARGET_VELOCITY_TOLERANCE = 50; // Place holder for now. Deadband for our velocity
  public static final int SHOOTER_MAX_VELOCITY_UNITS_100MILLISECONDS = 27000; // Derived from Phoneix Tuner
  public static final int SHOOTER_DESIRED_VELOCITY = 20000;
  public static final double SHOOTER_ANGLE_RED_ZONE = -16.0; // Angle of the Shooter/Limelight in the red zone
  public static final int SHOOTER_VELOCITY_RED_ZONE = 23000; // Shooting from 17'6" out or more from the target
  public static final double SHOOTER_ANGLE_BLUE_ZONE = -9.0; // Angle of the Shooter/Limelight in the red zone
  public static final int SHOOTER_VELOCITY_BLUE_ZONE = 18000; // Shooting from 12'6" to 17'6" from the target
  // public static final int SHOOTER_VELOCITY_BLUE_ZONE = 19500; // Shooting from
  // 12'6" to 17'6" from the target
  public static final int SHOOTER_DEFAULT_VELOCITY = 19500;
  public static final int SHOOTER_GREEN_ZONE_VELOCITY = 6450;
  public static final int SHOOTER_RED_ZONE_VELOCITY = 18000;  //18700;
  public static final int SHOOTER_BLUE_ZONE_VELOCITY = 17700; //18000;
  public static final int SHOOTER_YELLOW_ZONE_VELOCITY = 17700;
  public static final double SHOOTER_DEFAULT_P = .2; // Default value of kP for shooter PID
  public static final double SHOOTER_DEFAULT_I = .00; // Default value of kI for shooter PID
  public static final double SHOOTER_DEFAULT_D = 0; // Default value of kD for shooter PID
  public static final double SHOOTER_DEFAULT_F = .0485; // Default value of kF for shooter PID
  public static final double SHOOTER_GREEN_ZONE_P = .2;
  public static final double SHOOTER_GREEN_ZONE_I = 0;
  public static final double SHOOTER_GREEN_ZONE_D = 0;
  public static final double SHOOTER_GREEN_ZONE_F = .05;
  public static final double SHOOTER_RED_ZONE_P = 0.5;
  public static final double SHOOTER_RED_ZONE_I = 0;
  public static final double SHOOTER_RED_ZONE_D = 0;
  public static final double SHOOTER_RED_ZONE_F = .0485;
  public static final double SHOOTER_BLUE_ZONE_P = .5;
  public static final double SHOOTER_BLUE_ZONE_I = 0;
  public static final double SHOOTER_BLUE_ZONE_D = 0;
  public static final double SHOOTER_BLUE_ZONE_F = .048;
  public static final double SHOOTER_YELLOW_ZONE_P = .5;
  public static final double SHOOTER_YELLOW_ZONE_I = 0;
  public static final double SHOOTER_YELLOW_ZONE_D = 0;
  public static final double SHOOTER_YELLOW_ZONE_F = .05;

  public static final double SHOOTER_ANGLE_YELLOW_ZONE = 0.0; // Angle of the Shooter/Limelight in the red zone
  public static final int SHOOTER_VELOCITY_YELLOW_ZONE = 19000; // Shooting from 7'6" to 12'6" from the target
  public static final double SHOOTER_ANGLE_GREEN_ZONE = 0.0; // Angle of the Shooter/Limelight in the red zone
  public static final int SHOOTER_VELOCITY_GREEN_ZONE = 18500; // Shooting from less than 7'6"

  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
   * or 3. Only the first two (0,1) are visible in web-based configuration.
   */
  public static final int kSlotIdx_Velocity = 0;

  /**
   * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
   * we just want the primary one.
   */
  public static final int kPIDLoopIdx_Velocity = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and
   * report to DS if action fails.
   */
  public static final int kTimeoutMs_Velocity = 30;

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control
   * loop. kF: 1023 represents output value to Talon at 100%, 7200 represents
   * Velocity units at 100% output
   * 
   * kP kI kD kF Iz PeakOut
   */
  // public final static PIDGains kGains_Velocit = new PIDGains(0.25, 0.001, 20,
  // 1023.0 / 7200.0, 300, 1.00);

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control
   * loop. kF: 1023 represents output value to Talon at 100%, 6800 represents
   * Velocity units at 100% output Not all set of Gains are used in this project
   * and may be removed as desired.
   * 
   * kP kI kD kF Iz PeakOut
   */
  public final static Gains kGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
  public final static Gains kGains_Turning = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.00);
  public final static Gains kGains_Velocity = new Gains(0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50); // Need to tune the
                                                                                                     // 6800 # in
                                                                                                     // pheonix. Should
                                                                                                     // be the velocity
                                                                                                     // of our cim at
                                                                                                     // 100%. Do we
                                                                                                     // really want to
                                                                                                     // limit output to
                                                                                                     // .5?
  public final static Gains kGains_MotProf = new Gains(1.0, 0.0, 0.0, 1023.0 / 6800.0, 400, 1.00);

  public static final int PID_PRIMARY = 0;
  public static final int PID_TURN = 1;
  public static final int kTimeoutMs = 30;
  public static final int REMOTE_0 = 0;

  /*
   * Firmware currently supports slots [0, 3] and can be used for either PID Set
   */
  public final static int SLOT_0 = 0;
  public final static int SLOT_1 = 1;
  public final static int SLOT_2 = 2;
  public final static int SLOT_3 = 3;
  /* ---- Named slots, used to clarify code ---- */
  public final static int kSlot_Distanc = SLOT_0;
  public final static int kSlot_Turning = SLOT_1;
  public final static int kSlot_Velocit = SLOT_2;
  public final static int kSlot_MotProf = SLOT_3;

  public static final double kNeutralDeadband = 0.001;
public static final double DRIVETRAIN_TURN_THRESHOLD = 2;

// Galatic search constants
public static final double GS_B3_MAX = 0;
public static final double GS_C3_MAX = 7;
public static final double GS_C3_MIN = 0;
public static final double GS_D6_MAX = 18;
public static final double GS_D6_MIN = 7;


}
