// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretSetPositionCommand extends CommandBase {
  /** Creates a new TurretSetPositionCommand. */
    
  // Declare the Turret Subsystems
  TurretSubsystem m_turretSubsystem;

  //  Declare our desired position variable
  double m_desiredPosition;

  //  Declare our the position we are at when we at 

  public TurretSetPositionCommand(TurretSubsystem turretSubsystem, double desiredPosition) {

    m_turretSubsystem = turretSubsystem;
    m_desiredPosition = desiredPosition;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turretSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (m_turretSubsystem.cu)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
