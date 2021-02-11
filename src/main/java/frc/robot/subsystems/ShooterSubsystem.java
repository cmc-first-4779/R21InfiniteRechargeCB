// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  WPI_TalonFX shooterMotorMaster; //declare motors
  WPI_TalonFX shooterMotorSlave;


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

  shooterMotorMaster = new WPI_TalonFX(1); 
  shooterMotorSlave = new WPI_TalonFX(2);

  shooterMotorSlave.setInverted(true); //invert slave (might change)

  shooterMotorSlave.follow(shooterMotorMaster);//

  initMasterMotorController(shooterMotorMaster);
  initSlaveMotorController(shooterMotorSlave);

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private static void initMasterMotorController(WPI_TalonFX talon) {
    
    System.out.println("Initializing Master: " + talon);
    
    // Config to Factory Default
    talon.configFactoryDefault();
 
   
  }

   // Initialize a VictorSPX Motor controller and set our default settings.
   private static void initSlaveMotorController(WPI_TalonFX talon) {
    
    System.out.println("Initializing Slave: " + talon);
    
    // Config to Factory Default
    talon.configFactoryDefault();

  
  }


}
