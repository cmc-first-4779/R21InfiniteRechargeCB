// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleMotorSubsystem extends SubsystemBase {

  Spark motor;
  CANSparkMax sparkMax;
  TalonFX shooterMotor1;
  TalonFX shooterMotor2;

  double speed = 1;
  /** Creates a new SimpleMotorSubsystem. */
  public SimpleMotorSubsystem() {
    motor = new Spark(0);
    sparkMax = new CANSparkMax(10, MotorType.kBrushless);
    shooterMotor1 = new TalonFX(21);
    shooterMotor2 = new TalonFX(22);

    shooterMotor1.setInverted(true);
    shooterMotor2.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinforward() {
    // motor.set(speed);
     sparkMax.set(speed);
    shooterMotor1.set(ControlMode.PercentOutput, speed);
    shooterMotor2.set(ControlMode.PercentOutput, speed);
  }

  public void spinBackwards() {
    motor.set(speed * -1);
    sparkMax.set(speed * -.5);
  }

  public void stop(){
    motor.set(0);
    sparkMax.set(0);
    shooterMotor1.set(ControlMode.PercentOutput, 0);
    shooterMotor2.set(ControlMode.PercentOutput, 0);
  }
}
