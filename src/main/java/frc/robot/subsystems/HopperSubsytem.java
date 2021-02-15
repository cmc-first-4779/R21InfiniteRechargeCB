// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsytem extends SubsystemBase {
  Spark myMotor;

  /** Creates a new ExampleSubsystem. */
  public HopperSubsytem() {
    myMotor = new Spark(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void goForward(double speed){
    myMotor.set(speed);
  }
  public void goBackwards(double speed){
    myMotor.set(speed*-1);
  }
  public void stop(){
    myMotor.set(0);
  }
}
