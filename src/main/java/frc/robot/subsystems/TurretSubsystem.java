// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  //Declare our turret motor
  Spark turretMotor; 

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {

    turretMotor = new Spark(Constants.PWM_PORT_TURRET); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTurretMotorLeft(){ 
    turretMotor.set(-Constants.TURRET_MOTOR_SPEED);
  }

  public void setTurretMotorRight(){ 
    turretMotor.set(Constants.TURRET_MOTOR_SPEED);
  }

  //set motor speed when on
  public void setTurretMotorOff(){ 
    turretMotor.set(0);
  }

  //set motor speed when on
  public void setTurretMotorSpeed(double turretspeed){ 
    turretMotor.set(turretspeed);
  }

}
