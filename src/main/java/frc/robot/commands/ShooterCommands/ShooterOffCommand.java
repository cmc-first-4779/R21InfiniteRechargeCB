// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterOffCommand extends CommandBase {

  ShooterSubsystem m_shooterSubsystem;

  /** Creates a new ShooterOff. */
  public ShooterOffCommand(ShooterSubsystem shooterSubsystem) {
    
    m_shooterSubsystem = shooterSubsystem;
    
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override     
  public boolean isFinished() {
    return false;
  }
}
