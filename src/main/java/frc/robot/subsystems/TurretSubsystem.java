// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  //Declare our turret motor
  Spark turretMotor; 
  AnalogEncoder turretEncoder;

  //Declare our offset from the absolute zero position of the encoder
  //   This needs to be called after a reset and so we are putting it
  //   in the resetEncoder() method
  double turretPositionOffset;

  //Get current encoder location and angle
  double initialEncoderPositon;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    turretMotor = new Spark(Constants.PWM_PORT_TURRET_MOTOR); //Init our turret motor
    AnalogInput analogInput = new AnalogInput(0);  //  Use Analog port 0 for the encoder
    turretEncoder = new AnalogEncoder(analogInput);  //Init our turret encoder
    //THIS ENCODER IS AN ABSOLUTE ENCODER.  WE DO NOT RESET IT!!!
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //  Move the Turret Motor Counter Clockwise, but keep checking to see if it has passed our max
  //   Left Stop Encoder psotion
  public void setTurretMotorCounterClockwise(){ 
    if (getTurretEncoderPosition() > Constants.TURRET_LEFT_STOP_LOCATION_ENCODER_POSITION){
      //System.out.println("Current Position:  " + getTurretEncoderPosition());
      //  Negative speed to turn counter clockwise
      setTurretMotorSpeed(-1 * Constants.TURRET_MOTOR_SPEED);
    }
    else{
      //  Set speed to Zero..  Stop turret motor
      // System.out.println("Current Position:  " + getTurretEncoderPosition());
      stopTurret();
    }
  }

  //  Turn the Turret clockwise, but make sure it isn't beyond the right stop location
  public void setTurretMotorClockwise(){ 
    if (getTurretEncoderPosition() < Constants.TURRET_RIGHT_STOP_LOCATION_ENCODER_POSITION){
      //System.out.println("Current Position:  " + getTurretEncoderPosition());
      //  Positive speed to turn clockwise
      setTurretMotorSpeed(Constants.TURRET_MOTOR_SPEED);
    }
    else{
      //  Set speed to Zero..  Stop turret motor
      // System.out.println("Current Position:  " + getTurretEncoderPosition());
      stopTurret();
    }
  }

  //set motor speed when on.  Used also for limelight aiming.
  public void setTurretMotorSpeed(double turretspeed){
    //  If we are touching the left stop
    if (isTouchingLeftStop())
    {
      //  Set the InitalEncoderPosition to the left Manual Stop angle so that the scan will now
      //    turn clockwise after it exits this "if" statement
      if (!isTurningClockwise(turretspeed)){
        //  Since touched the right stop and we are turning clockwise, Set our initialEncoderPosition
        //    to the right stop encoder position so that the turett will now turn counterclockwise on the next loop
        turretspeed = 0;
      }
      System.out.println("We are touching the Left Stop, starting clockwise scan..");   
    }
      // Else if we are touching the right manual stop, set the inital stop angle so that the scan
      //   will not turn counterclockwise  
    else if (isTouchingRightStop())
    {
      if (isTurningClockwise(turretspeed)){
        //  Since touched the right stop and we are turning clockwise, Set our initialEncoderPosition
        //    to the right stop encoder position so that the turett will now turn counterclockwise on the next loop
        turretspeed = 0;
      }  
      System.out.println("We are touching the Right Stop, starting counter-clockwise scan..");  
    }
    turretMotor.set(turretspeed);
    SmartDashboard.putNumber("Turret Encoder Value:", getTurretEncoderPosition());
      
  }

  //  Stop the turret by setting the motor speed to zero
  public void stopTurret(){
    setTurretMotorSpeed(0);
  }

  //  Reset the Turret Encoders distance to zero.   We need to run this at the start of match
  public void resetTurretEncoder(){
    System.out.println("Resetting the Turret Encoder...");
    //Reset Encoder
    turretEncoder.reset();
    //  Store the offset position of the turret from the absolute zero position of the encoder
    turretPositionOffset = turretEncoder.getPositionOffset();
  }

  //  Return the Turret Encoder Position
  public double getTurretEncoderPosition(){
    //System.out.println("Current Position of Encoder:  " + turretEncoder.get());
    SmartDashboard.putNumber("Current Position:", turretEncoder.get());
    return turretEncoder.get();
  }  


  //  This method returns a boolean of TRUE if we are turning clockwise
  public boolean isTurningClockwise(double turretspeed){
    //  If the turret speed is greater than zero, it's going clockwise
    if (turretspeed >= 0){
      return true;
    }
    //  If our turret speed is less than zero, it's going counterclockwise
    else {
      return false;
    }
  }

    //  This routine will doublecheck our current location to see if we are touching the left manual stop
  //   It returns TRUE if we are...
  public boolean isTouchingLeftStop(){
    //  Get the current angle / position from the Turret Encoder
    double currentEncoderPosition = getTurretEncoderPosition();
    //  If the Encoder Angle is at the left manual stop, change our boolean to true...
    if (currentEncoderPosition <= Constants.TURRET_LEFT_STOP_LOCATION_ENCODER_POSITION){
        return true;
       }
    //  Else..   We are not touching the left manual stop   
    else {
        return false;
    }   
  }

//  This routine will doublecheck our current location to see if we are touching the Right manual stop
  //   It returns TRUE if we are...
  public boolean isTouchingRightStop(){
    //  Get the current angle / position from the Turret Encoder
    double currentEncoderPosition = getTurretEncoderPosition();
    //  If the Encoder Angle is at the right manual stop, change our boolean to true...
    if (currentEncoderPosition >= Constants.TURRET_RIGHT_STOP_LOCATION_ENCODER_POSITION){
        return true;
       }
    //  Else..   We are not touching the right manual stop   
    else {
        return false;
    }   
  }
  
}
