// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class StopTurretCommand extends CommandBase {
  /** Creates a new StopTurretCommand. */
  TurretSubsystem m_turretSubsystem;
  public StopTurretCommand(TurretSubsystem turretSubsystem) {
    m_turretSubsystem = turretSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turretSubsystem.stopTurret();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turretSubsystem.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
