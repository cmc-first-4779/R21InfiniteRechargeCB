// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class GetTurretEncoderPositionCommand extends CommandBase {
  /** Creates a new GetTurretEncoderPositionCommand. */
  TurretSubsystem m_turretSubsystem;
  public GetTurretEncoderPositionCommand(TurretSubsystem turretSubsystem) {
    m_turretSubsystem = turretSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  Return the encoder position
    m_turretSubsystem.getTurretEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
