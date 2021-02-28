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
    resetTurretEncoder();  //  Reset our encoder to zero when the subsystem is constructed.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTurretMotorCounterClockwise(){ 
    //  Negative speed to turn counter clockwise
    setTurretMotorSpeed(-1*Constants.TURRET_MOTOR_SPEED);
  }

  public void setTurretMotorClockwise(){ 
    setTurretMotorSpeed(Constants.TURRET_MOTOR_SPEED);
  }

  //set motor speed when on.  Used also for limelight aiming.
  public void setTurretMotorSpeed(double turretspeed){ 
    System.out.println("Encoder Angle:  " + getTurretAngleFromEncoder());
    System.out.println("Encoder: " + turretEncoder.get());
    System.out.println("Turret Speed:  " + turretspeed);
    
    //  If our current angle is less than the the right Manual Stop for the turret (0 degrees)
    //      I am in the deadzone
    if (getTurretAngleFromEncoder() <= Constants.TURRET_RIGHT_MANUAL_STOP_LOCATION_DEGREES)
    {
      //  If turret speed is negative, it's trying to turn counterclockwise past the right stop
      if (turretspeed < 0){
        System.out.println("I cannot turn counterclockwise because I would turn into the deadzone.");
        System.out.println("Not turning until I can turn the turret the other direction.");
      }
      //  The turretspeed is positive, let it turn clockwise
      else {
        System.out.println("I am turning clockwise");
        turretMotor.set(turretspeed);
      }
    }
    //  If our current angle is greater than or equal to the left Manual stop for the turret
    //     I am also in the deadzone here
    else if (getTurretAngleFromEncoder() >= Constants.TURRET_LEFT_MANUAL_STOP_LOCATION_DEGREES){
      //  if turret speed is positive, it's trying to turn clockwise passed the left stop
      if (turretspeed > 0){
        System.out.println("I cannot turn clockwise because I would turn into the deadzone.");
        System.out.println("Not turning until I can turn the turret the other direction.");
      }
      //  The turretspeed is negative, let it turn counterclockwise
      else {
        System.out.println("I am turning counterclockwise");
        turretMotor.set(turretspeed);
      }
    }
    else{
      //  Else..   I am not in the deadzone at all and I can turn either direction...
      System.out.println("I am no where near a manual stop, so I can turn either direction.");
      turretMotor.set(turretspeed);
    }
  }

  //  Stop the turret by setting the motor speed to zero
  public void stopTurret(){
    setTurretMotorSpeed(0);
  }

  //  Reset the Turret Encoders distance to zero.   We need to run this at the start of match
  public void resetTurretEncoder(){
    System.out.println("Resetting the Turret Encoder...");
    turretEncoder.reset();
  }

  //  Return the Turret Encoder Position
  public double getTurretEncoderPosition(){
    System.out.println("Current Position of Encoder:  " + turretEncoder.get());
    return turretEncoder.get();
  }  

  public double getTurretAngleFromEncoder(){
    double currentEncoderPosition = getTurretEncoderPosition();
    double angleFromEncoder = (currentEncoderPosition / Constants.TURRET_NUMBER_ENCODER_PULSES_PER_REVOLUTION) * 360;
    System.out.println("Current Angle from Encoder:  " + angleFromEncoder);
    return angleFromEncoder;
  }

}
