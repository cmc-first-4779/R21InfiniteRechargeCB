// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IntakeCommands.IntakeCellsCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Path_B_With_Intake extends ParallelRaceGroup {
  DriveTrainSubsystem m_driveTrainSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  /** Creates a new Path_B_With_Intake. */
  public Path_B_With_Intake(DriveTrainSubsystem dt, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_intakeSubsystem = intake;
    m_driveTrainSubsystem = dt;
    addCommands(
      new IntakeCellsCommand(m_intakeSubsystem),
      new Path_B_Drive(m_driveTrainSubsystem)
    );
  }
}
