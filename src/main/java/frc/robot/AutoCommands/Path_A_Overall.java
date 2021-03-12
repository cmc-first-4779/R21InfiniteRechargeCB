// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.RetractIntakeCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Path_A_Overall extends SequentialCommandGroup {
  IntakeSubsystem m_intakeSubsystem;
  DriveTrainSubsystem m_driveTrainSubsystem;
  /** Creates a new Path_A_Overall. */
  public Path_A_Overall(DriveTrainSubsystem dt, IntakeSubsystem intake) {
    m_intakeSubsystem = intake;
    m_driveTrainSubsystem = dt;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Path_A_With_Intake(m_driveTrainSubsystem, m_intakeSubsystem),
      new RetractIntakeCommand(m_intakeSubsystem)
    );
  }
}
