// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SimpleMotorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SimpleMotorSubsystem;

public class SpinBackwardsCommand extends CommandBase {
  private final SimpleMotorSubsystem simpleMotor;

  /** Creates a new SpinForwardCommand. */
  public SpinBackwardsCommand(SimpleMotorSubsystem sm) {
    this.simpleMotor = sm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(simpleMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    simpleMotor.spinBackwards();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    simpleMotor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
