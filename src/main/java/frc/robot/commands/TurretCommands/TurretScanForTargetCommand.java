// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.StaticConstants.BlingConstants;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretScanForTargetCommand extends CommandBase {
  /** Creates a new TurretScanForTargetCommand. */

    // Declare the DriveTrain and Limelight Subsystems
    TurretSubsystem m_turretSubsystem;
    LimelightSubsystem m_limelightSubsystem;
    double m_pipeline;
    BlingSubsystem m_blingSubsystem;
  
    // These will be used as we calculate turning and moving forward
    double turnTurretPower = 0;

    //Get current encoder location and angle
    double initialEncoderPositon;
   
  public TurretScanForTargetCommand(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem,
  double pipeline, BlingSubsystem blingSubsystem) {

      // Init our local variables
      m_turretSubsystem = turretSubsystem;
      m_limelightSubsystem = limelightSubsystem;
      m_pipeline = pipeline;
      m_blingSubsystem = blingSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turretSubsystem, m_limelightSubsystem, m_blingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the Limelight for Vision Mode
    m_limelightSubsystem.initLimelightforVision();
    // Ensure that the Pipeline is set for the correct target
    m_limelightSubsystem.setPipeline(m_pipeline);
    // Set BLING to LIME_GREEN as a visual cue for our driveteam that Vision is 
    // taking over the Turret
    m_blingSubsystem.setBlingPattern(BlingConstants.BLING_LIME);

    // get the position and angle values of the Turret Encoder when the command initially starts.
    initialEncoderPositon = m_turretSubsystem.getTurretEncoderPosition();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * First check to see if there is a target, if not turn 
     */
    boolean hasTarget = m_limelightSubsystem.hasTarget();
     //   If the Limelight can see the target...
    if (hasTarget) 
    {
      // Output to the Dashboard whether the LimeLight has a target
      System.out.println("LimeLight Has Target");
      SmartDashboard.putString("LimeLight Has Target", "TARGET ACQUIRED");
      //  Set the turn ower just enough to zero in on the target
      turnTurretPower = calculateTurn();
      System.out.println("Turn Value: " + turnTurretPower);
      //Turn the turret
      m_turretSubsystem.setTurretMotorSpeed(turnTurretPower);
    }
    //  Else..   If the Limelight can't see the target, we need to keep scanning
    else 
    {
      // Output to the Dashboard whether the LimeLight has a target
      System.out.println("LimeLight Has NO Target.  Seeking...");   
      SmartDashboard.putString("LimeLight Has Target", "NO TARGET");
      //  If we are touching the left Manual Stop...
      if (isTouchingLeftStop())
      {
        //  Set the InitalEncoderPosition to the left Manual Stop angle so that the scan will now
        //    turn clockwise after it exits this "if" statement
        if (!isTurningClockwise(turnTurretPower)){
          //  Since touched the right stop and we are turning clockwise, Set our initialEncoderPosition
          //    to the right stop encoder position so that the turett will now turn counterclockwise on the next loop
          initialEncoderPositon = m_turretSubsystem.getTurretEncoderPosition();
        }
        System.out.println("We are touching the Left Stop, starting clockwise scan..");   
      }
        // Else if we are touching the right manual stop, set the inital stop angle so that the scan
        //   will not turn counterclockwise  
      else if (isTouchingRightStop())
      {
        if (isTurningClockwise(turnTurretPower)){
          //  Since touched the right stop and we are turning clockwise, Set our initialEncoderPosition
          //    to the right stop encoder position so that the turett will now turn counterclockwise on the next loop
          initialEncoderPositon = m_turretSubsystem.getTurretEncoderPosition();
        }  
        System.out.println("We are touching the Right Stop, starting counter-clockwise scan..");  
      }
      //Set our seek power to our constant
      //  Because we have the manual stops in the turret, we have to be strategic in which way
      //    the turret turns to scan for a target..   If we are far to the left, then we scan to the right..
      //    if we started far to the right, then we scan to the left..
      if (initialEncoderPositon > 0)
      {
        //  Turn the turret clockwise if the turrent started to the left of the midpoint
        turnTurretPower = Constants.LIMELIGHT_SEEK_TURN_TURRET_POWER; 
        System.out.println("Turn Value: " + turnTurretPower);
      }
      else
      {
        //  Else..   We started to the right of the midpoint, so turn the turret counter clockwise
        turnTurretPower = -1 * Constants.LIMELIGHT_SEEK_TURN_TURRET_POWER; 
        System.out.println("Turn Value: " + turnTurretPower);
      }
    //FINALLY..   Turn the turret the correct direction based on the location and whether
    //  and whether the turret had been going clockwise or counter-clockwise..
    m_turretSubsystem.setTurretMotorSpeed(turnTurretPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


/**
   * Calculates how much to turn. Get's how far off in the x-axis we are from
   * limelight. If we are within acceptable range, then it returns 0.
   * 
   * @return the amount to turn
   */
  private double calculateTurn() {
    // Find out how much error we have on the X-axis
    double aim_error = m_limelightSubsystem.getTX();
    // If that error is too far to the right by our threshold / deadband, then
    // increase 'turn' to turn left by our min constant.
    if (aim_error > Constants.LIMELIGHT_AIMING_DEADBAND) {
      turnTurretPower = (Constants.LIMELIGHT_AIMING_kpAim * aim_error) + Constants.LIMELIGHT_AIMING_AIM_MIN_CMD;
    }
    // If that error is too far to the left by our threshold / deadband, then
    // increase 'turn' to turn right by our min constant.
    else if (aim_error < -Constants.LIMELIGHT_AIMING_DEADBAND) {
      turnTurretPower = (Constants.LIMELIGHT_AIMING_kpAim * aim_error) - Constants.LIMELIGHT_AIMING_AIM_MIN_CMD;
    }
    // If it is within the threshold / deadband, then DON'T TURN
    else {
      turnTurretPower = 0;
    }
    // Return the directon and amount we have to turn
    return turnTurretPower;
  }

  //  This routine will doublecheck our current location to see if we are touching the left manual stop
  //   It returns TRUE if we are...
  private boolean isTouchingLeftStop(){
    //  Get the current angle / position from the Turret Encoder
    double currentEncoderPosition = m_turretSubsystem.getTurretEncoderPosition();
    //  If the Encoder Angle is at the left manual stop, change our boolean to true...
    if (currentEncoderPosition <= Constants.TURRET_LEFT_MANUAL_STOP_LOCATION_ENCODER_PULSE){
        return true;
       }
    //  Else..   We are not touching the left manual stop   
    else {
        return false;
    }   
  }

//  This routine will doublecheck our current location to see if we are touching the Right manual stop
  //   It returns TRUE if we are...
  private boolean isTouchingRightStop(){
    //  Get the current angle / position from the Turret Encoder
    double currentEncoderPosition = m_turretSubsystem.getTurretEncoderPosition();
    //  If the Encoder Angle is at the right manual stop, change our boolean to true...
    if (currentEncoderPosition >= Constants.TURRET_RIGHT_MANUAL_STOP_LOCATION_ENCODER_PULSE){
        return true;
       }
    //  Else..   We are not touching the right manual stop   
    else {
        return false;
    }   
  }

  //  Returns true if we are turning clockwise
  private boolean isTurningClockwise(double turnTurrentPower){
    double m_turnTurretPower = turnTurrentPower;
    if (m_turnTurretPower > 0){
      return true;
    }
    else {
      return false;
    }
  }

}
