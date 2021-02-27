// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.HopperCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsytem;

public class HopperGoBackwardsCommand extends CommandBase {
  HopperSubsytem sub;
  /** Creates a new GoBackwardsCommand. */
  public HopperGoBackwardsCommand(HopperSubsytem sub) {
    // Use addRequirements() here to declare subsystem dependencies.
this.sub = sub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sub.goBackwards(.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
