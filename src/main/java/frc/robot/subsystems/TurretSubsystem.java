// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  //Declare our turret motor
  Spark turretMotor; 
  AnalogEncoder turretEncoder;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {

    turretMotor = new Spark(Constants.PWM_PORT_TURRET); 
    AnalogInput analogInput = new AnalogInput(0);
    turretEncoder = new AnalogEncoder(analogInput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTurretMotorCounterClockwise(){ 
    System.out.println("Encoder: " + turretEncoder.get());
    turretMotor.set(-Constants.TURRET_MOTOR_SPEED);
  }

  public void setTurretMotorClockwise(){ 
    System.out.println("Encoder: " + turretEncoder.get());
    turretMotor.set(Constants.TURRET_MOTOR_SPEED);
  }

  //  Used for the Limelight to aim..   Passes in a speed
  public void setTurretMotorAim(double speed){
    System.out.println("Encoder: " + turretEncoder.get());
    turretMotor.set(speed);
  }

  //set motor speed when on
  public void setTurretMotorOff(){ 
    System.out.println("Encoder: " + turretEncoder.get());
    turretMotor.set(0);
  }

  //set motor speed when on
  public void setTurretMotorSpeed(double turretspeed){ 
    System.out.println("Encoder: " + turretEncoder.get());
    turretMotor.set(turretspeed);
  }

  //  Stop the turret
  public void stopTurret(){
    turretMotor.set(0);
  }

  //  Reset the Turret Encoders distance to zero.   We need to run this at the start of match
  public void resetTurretEncoder(){
    turretEncoder.reset();
  }

  //  Return the Turret Encoder Position
  public double getTurretEncoderPosition(){
    return turretEncoder.get();
  }  

}
