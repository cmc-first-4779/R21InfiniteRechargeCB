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

  // Declare the Turret and Limelight Subsystems
  TurretSubsystem m_turretSubsystem;
  LimelightSubsystem m_limelightSubsystem;
  double m_pipeline;
  BlingSubsystem m_blingSubsystem;

  // These will be used as we calculate turning and moving forward
  double turnTurretPower = 0;

  // Get current encoder location and angle
  double initialEncoderPositon;

  // Counter to track which way its tracking
  int counter;

  double midEncoderPosition;

  public TurretScanForTargetCommand(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem,
      double pipeline, BlingSubsystem blingSubsystem) {

    // Init our local variables
    m_turretSubsystem = turretSubsystem;
    m_limelightSubsystem = limelightSubsystem;
    m_pipeline = pipeline;
    m_blingSubsystem = blingSubsystem;
    midEncoderPosition = (Constants.TURRET_LEFT_STOP_LOCATION_ENCODER_POSITION
        + Constants.TURRET_RIGHT_STOP_LOCATION_ENCODER_POSITION) / 2;

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

    // get the position and angle values of the Turret Encoder when the command
    // initially starts.
    initialEncoderPositon = m_turretSubsystem.getTurretEncoderPosition();

    // Turn the turret clockwise if the turrent started to the left of the midpoint

    if (initialEncoderPositon < midEncoderPosition) {
      turnTurretPower = Constants.LIMELIGHT_SEEK_TURN_TURRET_POWER;
    } else {
      turnTurretPower = -1 * Constants.LIMELIGHT_SEEK_TURN_TURRET_POWER;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * First check to see if there is a target, if not turn
     */
    boolean hasTarget = m_limelightSubsystem.hasTarget();
    // If the Limelight can see the target...
    if (hasTarget) {
      // Output to the Dashboard whether the LimeLight has a target
      System.out.println("LimeLight Has Target");
      SmartDashboard.putString("LimeLight Has Target", "TARGET ACQUIRED");
      end(false);
    }
    // Else.. If the Limelight can't see the target, we need to keep scanning
    else if ((turnTurretPower < 0) && (m_turretSubsystem.isTouchingLeftStop())) {
      turnTurretPower = -1 * turnTurretPower;
    } else if ((turnTurretPower >= 0) && (m_turretSubsystem.isTouchingRightStop())) {
      turnTurretPower = -1 * Constants.LIMELIGHT_SEEK_TURN_TURRET_POWER;
    }

    // FINALLY.. Turn the turret the correct direction based on the location and
    // whether
    // and whether the turret had been going clockwise or counter-clockwise..
    //
    // If the counter is even, we hit the end of the scanning range and need to
    // switch direction
    // (% 2 is a mod of 2, which looks for even numbers)

    m_turretSubsystem.setTurretMotorSpeed(turnTurretPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limelightSubsystem.hasTarget();
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

}
