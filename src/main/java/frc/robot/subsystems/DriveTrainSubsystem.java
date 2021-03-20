/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {



  // Declared talons and victors (talons-master, victor-slave)
  WPI_TalonFX leftMotorMaster; // Master
  WPI_TalonFX rightMotorMaster; // Master
  WPI_TalonFX leftMotorSlave; // Slave
  WPI_TalonFX rightMotorSlave; // Slave

  // Declared gyro
  public AHRS gyro;

  // Declared drive train
  DifferentialDrive myDrive;

  // Is my boolean set for my throttle if the arm or climber is deployed
  boolean isDriveTrainThrottled;

  /**
   * Constructor for DriveTrainSubsystem.
   */
  public DriveTrainSubsystem() {

    // Instantiate the motor controllers
    leftMotorMaster = new WPI_TalonFX(Constants.CAN_ADDRESS_FRONT_LEFT_DRIVE);
    rightMotorMaster = new WPI_TalonFX(Constants.CAN_ADDRESS_FRONT_RIGHT_DRIVE);
    leftMotorSlave = new WPI_TalonFX(Constants.CAN_ADDRESS_REAR_LEFT_DRIVE);
    rightMotorSlave = new WPI_TalonFX(Constants.CAN_ADDRESS_REAR_RIGHT_DRIVE);

    // Configure the Talons and Victors with our default settings using the
    // initMotorController method (below)
    initMotorController(leftMotorMaster);
    initMotorController(rightMotorMaster);
    initMotorController(leftMotorSlave);
    initMotorController(rightMotorSlave);

    // slaved rear motors to front motors
    leftMotorSlave.set(ControlMode.Follower, leftMotorMaster.getDeviceID());
    rightMotorSlave.set(ControlMode.Follower, rightMotorMaster.getDeviceID());

    // Generally one side of our motors will need to be inverted. Use Pheonix Tuner
    // to figure out which side it is based on wanting the the bot to move forward
    // when given a postive voltage. We can tell the slaves to just use the
    // inversion of their master
    // using setInverted(InvertType.FollowMaster)
    leftMotorMaster.setInverted(false);
    rightMotorMaster.setInverted(true);
    leftMotorSlave.setInverted(InvertType.FollowMaster);
    rightMotorSlave.setInverted(InvertType.FollowMaster);

    // Init the gyro/AHRS
    gyro = new AHRS(SPI.Port.kMXP);
    resetGyro(); //reset gyro

    // init the differential drive
    myDrive = new DifferentialDrive(leftMotorMaster, rightMotorMaster);

    // DifferentialDrive normally inverts the Right Side motors for you, since we
    // have already inverted one side of our motors, we set myDrive not to invert.
    myDrive.setRightSideInverted(false);

    // Setup the motors and encoders PID stuff
    configSimpleMM(); // This uses a simple Motion Magic that has each encoder use it's own encoder

    // Set initial throttle to false
    isDriveTrainThrottled = false;

  }

  /**
   * Sets up the talons to use their own Quad Encoders for their PID0. When setup
   * with this method, the user will have to set each talon to use Motion magic
   * when it's time to move.
   */
  private void configSimpleMM() {
    // Tell each talon to use Quad Encoder as their PID0
    leftMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.PID_PRIMARY,
        Constants.kTimeoutMs);
    rightMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.PID_PRIMARY,
        Constants.kTimeoutMs);

    // configPIDFValues(.0257, 0, .0125, 0, 0);
    configPIDFValues(.3, 0, 2, 0, 0);

    // Talons have 4 slots of PID variables and 2 PID indexes. Set the PID0 to use
    // Slot0
    leftMotorMaster.selectProfileSlot(0, 0);
    rightMotorMaster.selectProfileSlot(0, 0);

    configMotionCruiseAndAcceleration(Constants.DRIVETRAIN_MM_VELOCITY, Constants.DRIVETRAIN_MM_ACCELERATION); //8000 6000
    configPeakVelocities(1.0, -1.0);
    configAllowableError(0, 100);

    rightMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
  }

  private void configPIDFValues(double p, double i, double d, double f, int slot) {
    // Configure the PID settings for Slot0
    leftMotorMaster.config_kF(slot, f);
    leftMotorMaster.config_kP(slot, p);
    leftMotorMaster.config_kI(slot, i);
    leftMotorMaster.config_kD(slot, d);
    rightMotorMaster.config_kF(slot, f);
    rightMotorMaster.config_kP(slot, p);
    rightMotorMaster.config_kI(slot, i);
    rightMotorMaster.config_kD(slot, d);
  }

  private void configMotionCruiseAndAcceleration(int velocity, int acceleration) {

    // Motion Magic needs a CruiseVelocity and Acceleration that needs to be set.
    // The value is is sensor units/100ms. So a CIM Encoder has 80 units per
    // rotation and an AM Mag Encoder has 4096 units per rotation so this value will
    // be way differnent. A good way to figure this out is to use Phoenix Tuner to
    // run the motor at 100% and do a selftest snapshot to get the velocity which
    // will be in units/100ms. This is the max velocity the motor can do in it's
    // current configuration.
    leftMotorMaster.configMotionCruiseVelocity(velocity, 0);
    leftMotorMaster.configMotionAcceleration(acceleration, 0);
    rightMotorMaster.configMotionCruiseVelocity(velocity, 0);
    rightMotorMaster.configMotionAcceleration(acceleration, 0);

  }

  /**
   * Max out the peak output (for all modes). However you can limit the output of
   * a given PID object with configClosedLoopPeakOutput().
   * 
   * @param fwdMax
   * @param revMax
   */
  private void configPeakVelocities(double fwdMax, double revMax) {
    leftMotorMaster.configPeakOutputForward(fwdMax, Constants.kTimeoutMs);
    leftMotorMaster.configPeakOutputReverse(revMax, Constants.kTimeoutMs);
    rightMotorMaster.configPeakOutputForward(fwdMax, Constants.kTimeoutMs);
    rightMotorMaster.configPeakOutputReverse(revMax, Constants.kTimeoutMs);
  }

  private void configAllowableError(int slot, int allowedError) {
    // Configure the closeed loop error, which is how close the sensor has to be to
    // target to be successful.
    leftMotorMaster.configAllowableClosedloopError(0, 10, 3);
    rightMotorMaster.configAllowableClosedloopError(0, 10, 3);
  }

  /**
   * Checks to see if encoders are within range of target
   * 
   * @param target the end target of where they should be.
   * @return True if both encoders are within tolerance to target.
   */
  public boolean motionMagicOnTargetDrive(double target) {
    double tolerance = Constants.DRIVETRAIN_POSITION_TOLERANCE;
    double currentPos_L = leftMotorMaster.getSelectedSensorPosition();
    double currentPos_R = rightMotorMaster.getSelectedSensorPosition();

    return Math.abs(currentPos_L - target) < tolerance && Math.abs(currentPos_R - target) < tolerance;
  }

  @Override
  public void periodic() {
    // System.out.println("DriveTrain Sum: " +
    // frontRightDrive.getSelectedSensorPosition(0));
    // This method will be called once per scheduler run
  }

  // Initialize a TalonFX Motor controller and set our default settings.
  private static void initMotorController(WPI_TalonFX talon) {
    System.out.println("Initializing Talon SRX: " + talon);
    talon.configFactoryDefault();
    talon.setNeutralMode(NeutralMode.Brake); // Neutral Mode is Brake
    talon.neutralOutput();
    talon.setSensorPhase(false);
    talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    talon.configNominalOutputForward(0.0, 0);
    talon.configNominalOutputReverse(0.0, 0);
    talon.configClosedloopRamp(0.5, 0);
  }


  // Our Arcade drive method
  public void arcadeDrive(double move, double turn) {
      myDrive.arcadeDrive(move, turn);
      //curvatureDrive(move, turn, false);
    //}
  }

  public void curvatureDrive(double move, double turn, boolean isQuickTurn) {
    myDrive.curvatureDrive(move, turn, isQuickTurn);
  }

  // This method exists so that our commands can call on the subsystem to reset
  // the Gyro. Usually, we call this each time we initialize a drivetrain command.
  public void resetGyro() {
    System.out.println("Resetting NAV-X Gyro");
    gyro.reset();
  }

  // Returns the angle of the gyro
  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public double getEncoderPosition() {
    // We needed to add this to error check just in case one of our encoders is out
    // of service due to a bad connection or sliced wire. These if/else commands
    // make sure that each encoder is giving positive values >= 3 before performing
    // an average. If one
    // of the encoders is < 3, then it will only return a reading from the working
    // encoder.
    if (Math.abs(leftMotorMaster.getSelectedSensorPosition()) < 3) {
      System.out.println("FRONT LEFT DRIVETRAIN ENCODER IS NOT WORKING!!!");
      return rightMotorMaster.getSelectedSensorPosition();
    } else if (Math.abs(rightMotorMaster.getSelectedSensorPosition()) < 3) {
      System.out.println("FRONT RIGHT DRIVETRAIN ENCODER IS NOT WORKING!!!");
      return -leftMotorMaster.getSelectedSensorPosition();
    } else {
      // Average our two rotary encoders together to account for slippage and turning.
      return (-leftMotorMaster.getSelectedSensorPosition() + rightMotorMaster.getSelectedSensorPosition()) / 2;
    }
  }

  public void zeroEncoders() {
    int kTimeoutMs = 30;
    leftMotorMaster.getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
    rightMotorMaster.getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
    System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
  }

  public void driveStraight(int distance) {
    System.out.println("Driving straight");
    // frontRightDrive.set(ControlMode.MotionMagic, distance, DemandType.AuxPID,
    // frontRightDrive.getSelectedSensorPosition(Constants.PID_TURN));
    rightMotorMaster.set(ControlMode.Position, distance, DemandType.AuxPID,
        rightMotorMaster.getSelectedSensorPosition(1));
    leftMotorMaster.follow(rightMotorMaster, FollowerType.AuxOutput1);
  }

  /**
   * Uses motion magic to move the given distance
   * 
   * @param distance how many Sensor units to move.
   */
  public void simpleMM(double distance) {
    myDrive.setSafetyEnabled(false);
    // distance = SmartDashboard.getNumber("MM Distance", 1000);
    leftMotorMaster.set(ControlMode.MotionMagic, distance);
    rightMotorMaster.set(ControlMode.MotionMagic, distance);
  }

  /**
   * Sets the motor safety on or off.
   */
  public void setMotorSafety(boolean turnOn) {
    myDrive.setSafetyEnabled(turnOn);
  }

  // In this method we can set the neutral mode for all of the DriveTrain motor
  // controllers
  // Options: BRAKE where the motor controllers do not allow the robot to coast
  // COAST where the motor controllers allow the robot to coast
  // We did this to make the robot easier to push around the field when it is not
  // in auton or teleop
  public void setNeutralMode(NeutralMode mode) {
    leftMotorMaster.setNeutralMode(mode);
    leftMotorSlave.setNeutralMode(mode);
    rightMotorMaster.setNeutralMode(mode);
    rightMotorSlave.setNeutralMode(mode);
  }

public void stopMotors() {
  rightMotorMaster.stopMotor();
  leftMotorMaster.stopMotor();
}

}
