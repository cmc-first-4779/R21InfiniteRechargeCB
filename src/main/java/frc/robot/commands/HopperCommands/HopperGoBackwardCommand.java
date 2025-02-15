// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.HopperCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

public class HopperGoBackwardCommand extends CommandBase {
    HopperSubsystem m_hopperSubsystem;
    /** Creates a new GoBackwardsCommand. */
 
    public HopperGoBackwardCommand(HopperSubsystem hopperSubsystem) {
      m_hopperSubsystem = hopperSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_hopperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hopperSubsystem.goBackward(Constants.HOPPER_MOTOR_BACKWARD_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopperSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
