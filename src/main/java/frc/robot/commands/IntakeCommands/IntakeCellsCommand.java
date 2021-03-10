// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCellsCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;

  Timer myTimer;

  public IntakeCellsCommand(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    myTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.extendIntake();
    myTimer.reset();
    myTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (myTimer.get() >= 1) {
      m_intakeSubsystem.startIntakeRoller(Constants.INTAKE_MOTOR_SPEED);
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntakeRoller();
    m_intakeSubsystem.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
