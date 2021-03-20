// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  //  Declare our SparkMax controller on the CAN for the intake
  CANSparkMax intakeMotor;

  //  Declare our solenoid for the intake
  DoubleSolenoid intakeDoubleSolenoid;

  public IntakeSubsystem() {
    //  Init the motor and solenoid on the right ports
    intakeMotor = new CANSparkMax(Constants.CAN_ADDRESS_INTAKE, MotorType.kBrushless);
    intakeDoubleSolenoid = new DoubleSolenoid(Constants.PCM_PORT_INTAKE_EXTEND, Constants.PCM_PORT_INTAKE_RETRACT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  //  Start the Intake Motor on the inputed speed
  public void startIntakeRoller(double speed) {
    intakeMotor.set(speed);
  }

  //  Reverse the intake motor on the inputed speed.
  //   NOTE:  The value of speed should be positive as we multiply it by (-1) below
  public void reverseIntakeRoller(double speed) {
    intakeMotor.set(speed*-1);
  }

  //  Stop the roller by setting the speed to zero
  public void stopIntakeRoller(){
    intakeMotor.set(0);
  }

  //  Extend the intake by pushing the solenoid out
  public void extendIntake(){
    intakeDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  //  Retract the intake by releasing the air pressure
  public void retractIntake(){
    intakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

}