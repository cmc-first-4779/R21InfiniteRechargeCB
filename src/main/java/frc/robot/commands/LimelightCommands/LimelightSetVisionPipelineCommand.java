/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StaticConstants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightSetVisionPipelineCommand extends CommandBase {
  /**
   * Creates a new LimelightSetVisionPipelineCommand.
   */

  //Declare our Subsystem
  LimelightSubsystem m_limelightSubsystem;
  //Declare our vision pipeline
  double m_pipeline;

  public LimelightSetVisionPipelineCommand(LimelightSubsystem limelightSubsystem, double pipeline) {
    m_limelightSubsystem = limelightSubsystem;
    m_pipeline = pipeline;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Turn the LED on for the Camera
    m_limelightSubsystem.setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_ON);
    //Turn the Camera mode to Vision
    m_limelightSubsystem.setCameraMode(LimelightConstants.LIMELIGHT_CAMMODE_VISION);
    //Set the pipeline to the desired pipeline
    m_limelightSubsystem.setPipeline(m_pipeline);
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
