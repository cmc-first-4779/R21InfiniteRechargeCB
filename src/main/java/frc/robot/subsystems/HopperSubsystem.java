// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
  CANSparkMax myMotor;

  /** Creates a new ExampleSubsystem. */
  public HopperSubsystem() {
    //myMotor = new CANSparkMax(10, MotorType.kBrushless);
    myMotor = new CANSparkMax(Constants.CAN_ADDRESS_HOPPER, MotorType.kBrushless);
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
  public void goBackward(double speed){
    myMotor.set(speed*-1);
  }
  public void stop(){
    myMotor.set(0);
  }
}
