// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleMotorSubsystem extends SubsystemBase {

  Spark motor;
  double speed = .5;
  /** Creates a new SimpleMotorSubsystem. */
  public SimpleMotorSubsystem() {
    motor = new Spark(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinforward() {
    motor.set(speed);
  }

  public void spinBackwards() {
    motor.set(speed * -1);
  }

  public void stop(){
    motor.set(0);
  }
}
