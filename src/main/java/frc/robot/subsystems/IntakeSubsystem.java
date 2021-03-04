// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//pneumatic cylinder to push out to pick up powercells
//single motor to run rollers
//No encoder needed
//Spark motor controller due to simplicity
//Will tune speed of rollers when testing


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax intakeMotor;
  DoubleSolenoid intakeDoubleSolenoid;

  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.CAN_ADDRESS_INTAKE, MotorType.kBrushless);
    intakeDoubleSolenoid = new DoubleSolenoid(Constants.PCM_PORT_INTAKE_EXTEND, Constants.PCM_PORT_INTAKE_RETRACT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void startIntakeRoller(double speed) {
    intakeMotor.set(speed);
  }

  public void reverseIntakeRoller(double speed) {
    intakeMotor.set(speed*-1);
  }

  public void stopIntakeRoller(){
    intakeMotor.set(0);
  }

  public void extendIntake(){
    intakeDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractIntake(){
    intakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

}