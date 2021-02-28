/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.StaticConstants.BlingConstants;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class SeekAndAimCommand extends CommandBase {
  /**
   * Creates a new SeekAndAimCommand.
   */

  // Declare the DriveTrain and Limelight Subsystems
  DriveTrainSubsystem m_driveTrainSubsystem;
  LimelightSubsystem m_limelightSubsystem;
  double m_pipeline;
  BlingSubsystem m_blingSubsystem;

  // These will be used as we calculate turning and moving forward
  double turn;
  double move;
  boolean aimAndMove = false;

  public SeekAndAimCommand(DriveTrainSubsystem driveTrainSubsystem, LimelightSubsystem limelightSubsystem,
      double pipeline, BlingSubsystem blingSubsystem) {

    // Init our local variables
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_limelightSubsystem = limelightSubsystem;
    m_pipeline = pipeline;
    m_blingSubsystem = blingSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrainSubsystem, m_limelightSubsystem, m_blingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the Limelight for Vision Mode
    m_limelightSubsystem.initLimelightforVision();
    // Ensure that the Pipeline is set for the correct target
    m_limelightSubsystem.setPipeline(m_pipeline);
    // Set BLING to LIME_GREEN as a visual cue for our driveteam that Vision is
    // taking over the DriveTrain
    m_blingSubsystem.setBlingPattern(BlingConstants.BLING_LIME);

    // Set our turn and move variables to zero
    // turn = 0;
    // move = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * First check to see if there is a target, if not turn Then make sure we are on
     * target Then make sure we are the proper distance from target
     */
    boolean hasTarget = m_limelightSubsystem.hasTarget();

    if (hasTarget) {
      // Output to the Dashboard whether the LimeLight has a target
      System.out.println("LimeLight Has Target");
      SmartDashboard.putString("LimeLight Has Target", "TARGET ACQUIRED");
      turn = calculateTurn();
      System.out.println("Turn Value: " + turn);
      if (aimAndMove) {
        move = calculateMove();
        System.out.println("Move Value: " + move);
      } else { // only want to move if we are on target already
        if (turn == 0) {
          move = calculateMove();
          System.out.println("Move Value: " + move);
        }
      }
    } else {
      // Output to the Dashboard whether the LimeLight has a target
      SmartDashboard.putString("LimeLight Has Target", "NO TARGET");
      move = 0;
      turn = Constants.LIMELIGHT_SEEK_TURN_POWER;
    }

    // Send the "move" and "turn" values into arcade drive
    m_driveTrainSubsystem.arcadeDrive(move, turn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //  Set our bling back to default
    m_blingSubsystem.setBlingPattern(Constants.BLING_DEFAULT);
  }

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
      turn = (Constants.LIMELIGHT_AIMING_kpAim * aim_error) + Constants.LIMELIGHT_AIMING_AIM_MIN_CMD;
    }
    // If that error is too far to the left by our threshold / deadband, then
    // increase 'turn' to turn right by our min constant.
    else if (aim_error < -Constants.LIMELIGHT_AIMING_DEADBAND) {
      turn = (Constants.LIMELIGHT_AIMING_kpAim * aim_error) - Constants.LIMELIGHT_AIMING_AIM_MIN_CMD;
    }
    // If it is within the threshold / deadband, then DON'T TURN
    else {
      turn = 0;
    }
    // Return the directon and amount we have to turn
    return turn;
  }

  /**
   * Calculates the amount to drive. Get's how far away we are from target based
   * on x-axis from limelight. If we are within acceptable range, then it returns
   * 0
   * 
   * @return
   */
  private double calculateMove() {
    double distance_error = m_limelightSubsystem.getTY(); // How far are we away from the target (LIMELIGHT Y-axis)

    // If we are too far away.. Calculate our move to go forward and get closer..
    if (distance_error > Constants.LIMELIGHT_AIMING_DISTANCE_TOLERANCE) {
      move = (Constants.LIMELIGHT_AIMING_kpDist * distance_error) + Constants.LIMELIGHT_AIMING_MOVE_MIN_CMD;
    }
    // Else If we are too close.. Calculate our move to back up..
    else if (distance_error < -Constants.LIMELIGHT_AIMING_DISTANCE_TOLERANCE) {
      move = (Constants.LIMELIGHT_AIMING_kpDist * distance_error) - Constants.LIMELIGHT_AIMING_MOVE_MIN_CMD;
    }
    // Else don't move..
    else {
      move = 0;
    }
    return -move;
  }
}
