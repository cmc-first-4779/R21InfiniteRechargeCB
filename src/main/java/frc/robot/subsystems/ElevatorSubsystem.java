// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  Spark myMoter;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    myMoter = new Spark(0);
    // This method will be called once per scheduler run
  }
  public void goForward(double speed) {
    myMoter.set(speed);
  }
  public void goBackwards(double speed) {
    myMoter.set(speed*=1);
  }
}
