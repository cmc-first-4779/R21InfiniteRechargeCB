/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightInitForVisionCommand extends CommandBase {
  /**
   * Creates a new LimelightInitForVisionCommand.
   */

   LimelightSubsystem m_limelightSubsystem;

  public LimelightInitForVisionCommand(LimelightSubsystem limelightSubsystem) {

    m_limelightSubsystem = limelightSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelightSubsystem.initLimelightforVision();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
