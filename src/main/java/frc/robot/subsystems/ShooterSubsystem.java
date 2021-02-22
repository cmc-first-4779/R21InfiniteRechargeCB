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

  WPI_TalonFX shooterMotorMaster; //declare motors
  WPI_TalonFX shooterMotorSlave;

  private double m_desiredSpeed;

  private Boolean isShooterUpToSpeed;


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

  shooterMotorMaster = new WPI_TalonFX(1); 
  shooterMotorSlave = new WPI_TalonFX(2);

  shooterMotorSlave.setInverted(true); //invert slave (might change)

  shooterMotorSlave.follow(shooterMotorMaster);//

  initMasterMotorController(shooterMotorMaster);
  initSlaveMotorController(shooterMotorSlave);

  isShooterUpToSpeed = false;

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private static void initMasterMotorController(WPI_TalonFX falcon) {
      
    System.out.println("Initializing Master: " + falcon);
    
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

   private void configPIDFValues(double p, double i, double d, double f, int slot) {
    // Configure the PID settings for Slot0
    shooterMotorMaster.config_kF(slot, f);
    shooterMotorMaster.config_kP(slot, p);
    shooterMotorMaster.config_kI(slot, i);
    shooterMotorMaster.config_kD(slot, d);
  }

   public void setConstantVelocityLow() {
    configPIDFValues(1, 0, 0, 0.02, 0);  //STILL NEED TO GET THESE VALUES
   
    shooterMotorMaster.set(ControlMode.Velocity, 8000);

    m_desiredSpeed = 8000;
  }

  // Stop our shooter to stop the shooter
  public void stopMotor() {
    shooterMotorMaster.stopMotor();
  }

  // Gets the current velocity of the shooter master motor
  public double getShooterVelocity() {
    return shooterMotorMaster.getSelectedSensorVelocity();
  }

    //   Sets the desired speeed for our Shooter fly wheels
  public void setDesiredSpeed(Double desiredSpeed){
    m_desiredSpeed = desiredSpeed;
  }

  //   Gets the Desired Speed for the Shooter Fly Wheels
  public double getDesiredSpeed(){
    return m_desiredSpeed;
  }

  public boolean isUpToSpeed(){
    isShooterUpToSpeed = Math.abs(shooterMotorMaster.getSelectedSensorVelocity() - m_desiredSpeed) < 350;
    SmartDashboard.putBoolean("Is Shooter Up to Speed", isShooterUpToSpeed);
    System.out.println("Shooting Speed:  " + shooterMotorMaster.getSelectedSensorVelocity());
    System.out.println("Desired Speed:  " + m_desiredSpeed);
    return isShooterUpToSpeed;
  }

  
  }

