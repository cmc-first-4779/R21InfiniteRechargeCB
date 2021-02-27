// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  Spark elevatorMotor;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor = new Spark(Constants.PWM_PORT_ELEVATOR_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  //  Elevator go up method
  public void goUp(double speed) {
    elevatorMotor.set(speed);
  }
  
  //  Elevator go down method
  public void goDown(double speed) {
    // NOTE:   The speed is multiplied by -1 to spin the motor in reverse
    elevatorMotor.set(-1*speed);
  }

  //  Elevator stop method
  public void stopElevator() {
    elevatorMotor.set(0);
  }
}
