// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//pneumatic cylinder to push out to pick up powercells
//single motor to run rollers
//No encoder needed
//Spark motor controller due to simplicity
//Will tune speed of rollers when testing


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  Spark intakeMotor;

  public IntakeSubsystem() {
    intakeMotor = new Spark(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void intakeOn(double speed) {
    intakeMotor.set(speed);
  }

  public void intakeBackward(double speed) {
    intakeMotor.set(speed*-1);
  }

}