// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands.GalacticSearchCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IntakeCommands.IntakeCellsCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchWithIntake extends ParallelRaceGroup {
  DriveTrainSubsystem m_driveTrainSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  LimelightSubsystem m_limLimelightSubsystem;
  /** Creates a new GalacticSearchWithIntake. */
  public GalacticSearchWithIntake(DriveTrainSubsystem dt, IntakeSubsystem intake, LimelightSubsystem lm) {
    m_driveTrainSubsystem = dt;
    m_intakeSubsystem = intake;
    m_limLimelightSubsystem = lm;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeCellsCommand(m_intakeSubsystem),
      new GalacticSearchCommand(m_driveTrainSubsystem, m_limLimelightSubsystem)
    );
  }
}
