// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  WPI_TalonFX shooterMotorMaster; // declare motor master
  WPI_TalonFX shooterMotorSlave;  //  declare motor slave

  private double m_desiredSpeed;  //  declare the desiredSpeed variable

  private Boolean isShooterUpToSpeed;  // declare our variable for is the shooter up to speed

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // setupSmartDashboard();

    //  Init the motor controllers as Falcons
    shooterMotorMaster = new WPI_TalonFX(Constants.CAN_ADDRESS_SHOOTER_MASTER);
    shooterMotorSlave = new WPI_TalonFX(Constants.CAN_ADDRESS_SHOOTER_SLAVE);

    //  Set our default settings
    initMasterMotorController(shooterMotorMaster);
    initSlaveMotorController(shooterMotorSlave);

    //  Invert the master from the slave so that we are spinning in the right direction
    shooterMotorSlave.setInverted(false); // invert slave (might change)
    shooterMotorMaster.setInverted(true); // invert master (might change)

    //  Have the slave follow the master
    shooterMotorSlave.follow(shooterMotorMaster);//

    //  Set the initial value of our up to speed variable to "false"
    isShooterUpToSpeed = false;

    //  Set our desired speed to the speed in the Constants class
    //   NOTE:   We may not use this later..
    m_desiredSpeed = Constants.SHOOTER_DEFAULT_VELOCITY;

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //   This is there we set up the Falcon Master controller settings so that it is in a 
  //     known state
  private static void initMasterMotorController(WPI_TalonFX falcon) {

    System.out.println("Initializing Master: " + falcon);

    // Config to Factory Default
    falcon.configFactoryDefault();
    falcon.setNeutralMode(NeutralMode.Coast); // Neutral Mode is "Coast"
    falcon.setSensorPhase(true);
    falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx_Velocity,
        Constants.kTimeoutMs_Velocity);
    falcon.configNominalOutputForward(0, Constants.kTimeoutMs_Velocity);
    falcon.configNominalOutputReverse(0, Constants.kTimeoutMs_Velocity);
    falcon.configPeakOutputForward(1, Constants.kTimeoutMs_Velocity);
    falcon.configPeakOutputReverse(1, Constants.kTimeoutMs_Velocity);
  }

   //   This is there we set up the Falcon Slave controller settings so that it is in a 
  //     known state
  private static void initSlaveMotorController(WPI_TalonFX falcon) {
    System.out.println("Initializing Slave: " + falcon);
    // Config to Factory Default
    falcon.configFactoryDefault();
    falcon.setNeutralMode(NeutralMode.Coast); // Neutral Mode is "Coast"
    falcon.setSensorPhase(true);
    falcon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx_Velocity,
        Constants.kTimeoutMs_Velocity);
    falcon.configNominalOutputForward(0, Constants.kTimeoutMs_Velocity);
    falcon.configNominalOutputReverse(0, Constants.kTimeoutMs_Velocity);
    falcon.configPeakOutputForward(1, Constants.kTimeoutMs_Velocity);
    falcon.configPeakOutputReverse(-1, Constants.kTimeoutMs_Velocity);
  }

  //  Set up our PID Values for the motor controller
  private void configPIDFValues(double p, double i, double d, double f, int slot) {
    // Configure the PID settings for Slot0
    shooterMotorMaster.config_kF(slot, f);
    shooterMotorMaster.config_kP(slot, p);
    shooterMotorMaster.config_kI(slot, i);
    shooterMotorMaster.config_kD(slot, d);
  }

  //  This sets the target shooter velocity for the controllers based off of Constant
  public void setConstantVelocity() {
    configPIDFValues(Constants.SHOOTER_DEFAULT_P, Constants.SHOOTER_DEFAULT_I, Constants.SHOOTER_DEFAULT_D, Constants.SHOOTER_DEFAULT_F, 0); // STILL NEED TO GET THESE VALUES
    shooterMotorMaster.set(ControlMode.Velocity, m_desiredSpeed);
  }

  //  This sets the target velocity for the shooter controllers based on an input and a button press
  public void setConstantVelocityFromInput(int buttonNumber, int velocity) {
    //  Initially set our PID Values to our constants, those these may change based on how far away
    //    we are shooting from.
    double kP = Constants.SHOOTER_DEFAULT_P;
    double kI = Constants.SHOOTER_DEFAULT_I;
    double kD = Constants.SHOOTER_DEFAULT_D;
    double kF = Constants.SHOOTER_DEFAULT_F;

    //  Set our desired speed for now, but this will change based on how far we are shooting from
    m_desiredSpeed = velocity;

    //  This SWITCH statement will change the PID values and velocity based on what button is pushed on
    //    the joystick
    switch (buttonNumber) {
      case 0: //default case
        configPIDFValues(kP, kI, kD, kF, 0); // STILL NEED TO GET THESE VALUES
        break;
      case 1:
        kP = SmartDashboard.getNumber("GreenZone_P", Constants.SHOOTER_DEFAULT_P);
        kI = SmartDashboard.getNumber("GreenZone_I", Constants.SHOOTER_DEFAULT_I);
        kD = SmartDashboard.getNumber("GreenZone_D", Constants.SHOOTER_DEFAULT_D);
        kF = SmartDashboard.getNumber("GreenZone_F", Constants.SHOOTER_DEFAULT_F);
        break;
      case 2:
        kP = SmartDashboard.getNumber("RedZone_P", Constants.SHOOTER_DEFAULT_P);
        kI = SmartDashboard.getNumber("RedZone_I", Constants.SHOOTER_DEFAULT_I);
        kD = SmartDashboard.getNumber("RedZone_D", Constants.SHOOTER_DEFAULT_D);
        kF = SmartDashboard.getNumber("RedZone_F", Constants.SHOOTER_DEFAULT_F);
        break;
      case 3:
        kP = SmartDashboard.getNumber("BlueZone_P", Constants.SHOOTER_DEFAULT_P);
        kI = SmartDashboard.getNumber("BlueZone_I", Constants.SHOOTER_DEFAULT_I);
        kD = SmartDashboard.getNumber("BlueZone_D", Constants.SHOOTER_DEFAULT_D);
        kF = SmartDashboard.getNumber("BlueZone_F", Constants.SHOOTER_DEFAULT_F);
        break;
      case 4:
        kP = SmartDashboard.getNumber("YellowZone_P", Constants.SHOOTER_DEFAULT_P);
        kI = SmartDashboard.getNumber("YellowZone_I", Constants.SHOOTER_DEFAULT_I);
        kD = SmartDashboard.getNumber("YellowZone_D", Constants.SHOOTER_DEFAULT_D);
        kF = SmartDashboard.getNumber("YellowZone_F", Constants.SHOOTER_DEFAULT_F);
        break;
      default:
        kP = Constants.SHOOTER_DEFAULT_P;
        kI = Constants.SHOOTER_DEFAULT_I;
        kD = Constants.SHOOTER_DEFAULT_D;
        kF = Constants.SHOOTER_DEFAULT_F;
    }
    configPIDFValues(kP, kI, kD, kF, 0);
    shooterMotorMaster.set(ControlMode.Velocity, m_desiredSpeed);
  }

  //  Set the velocity from an input
  public void setConstantVelocityFromInput(int velocity) {
    configPIDFValues(.1, 0, 0, 0.05, 0);
    // Added these steps to allow us to set the constant velocity based on passed
    // integer.
    m_desiredSpeed = velocity;
    shooterMotorMaster.set(ControlMode.Velocity, velocity);
  }

  // Stop our shooter to stop the shooter
  public void stopMotor() {
    shooterMotorMaster.stopMotor();
  }

  // Gets the current velocity of the shooter master motor
  public double getShooterVelocity() {
    return shooterMotorMaster.getSelectedSensorVelocity();
  }

  // Sets the desired speeed for our Shooter fly wheels
  public void setDesiredSpeed(Double desiredSpeed) {
    m_desiredSpeed = desiredSpeed;
  }

  // Gets the Desired Speed for the Shooter Fly Wheels
  public double getDesiredSpeed() {
    return m_desiredSpeed;
  }

  //   Returns a boolean on whether the flywheels are up to speed
  public boolean isUpToSpeed() {
    isShooterUpToSpeed = Math.abs(
        shooterMotorMaster.getSelectedSensorVelocity() - m_desiredSpeed) < Constants.SHOOTER_TARGET_VELOCITY_TOLERANCE;
    SmartDashboard.putBoolean("Is Shooter Up to Speed", isShooterUpToSpeed);
    System.out.println("Shooting Speed:  " + shooterMotorMaster.getSelectedSensorVelocity());
    System.out.println("Desired Speed:  " + m_desiredSpeed);
    return isShooterUpToSpeed;
  }

  //  Setup the Shuffleboard so that we can tune for calibrating the shooter.
  //    Only used during calibration..   Otherwise these values are hard-coded.
  private void setupSmartDashboard() {
    addPIDFToSmartDashboard("Default", Constants.SHOOTER_DEFAULT_VELOCITY, Constants.SHOOTER_DEFAULT_P, Constants.SHOOTER_DEFAULT_I, Constants.SHOOTER_DEFAULT_D, Constants.SHOOTER_DEFAULT_F);
    addPIDFToSmartDashboard("Green", Constants.SHOOTER_GREEN_ZONE_VELOCITY, Constants.SHOOTER_GREEN_ZONE_P, Constants.SHOOTER_GREEN_ZONE_I, Constants.SHOOTER_GREEN_ZONE_D, Constants.SHOOTER_GREEN_ZONE_F);
    addPIDFToSmartDashboard("Red", Constants.SHOOTER_RED_ZONE_VELOCITY, Constants.SHOOTER_RED_ZONE_P, Constants.SHOOTER_RED_ZONE_I, Constants.SHOOTER_RED_ZONE_D, Constants.SHOOTER_RED_ZONE_F);
    addPIDFToSmartDashboard("Blue", Constants.SHOOTER_BLUE_ZONE_VELOCITY, Constants.SHOOTER_BLUE_ZONE_P, Constants.SHOOTER_BLUE_ZONE_I, Constants.SHOOTER_BLUE_ZONE_D, Constants.SHOOTER_BLUE_ZONE_F);
    addPIDFToSmartDashboard("Yellow", Constants.SHOOTER_YELLOW_ZONE_VELOCITY, Constants.SHOOTER_YELLOW_ZONE_P, Constants.SHOOTER_YELLOW_ZONE_I, Constants.SHOOTER_YELLOW_ZONE_D, Constants.SHOOTER_YELLOW_ZONE_F);
  }

  private void addPIDFToSmartDashboard (String zone, int velocity, double p, double i, double d, double f) {
    SmartDashboard.putNumber(zone + "ZoneVelocity", velocity);
    SmartDashboard.putNumber(zone +"Zone_P", p);
    SmartDashboard.putNumber(zone +"Zone_I", i);
    SmartDashboard.putNumber(zone +"Zone_D", d);
    SmartDashboard.putNumber(zone +"Zone_F", f);
  }

}
