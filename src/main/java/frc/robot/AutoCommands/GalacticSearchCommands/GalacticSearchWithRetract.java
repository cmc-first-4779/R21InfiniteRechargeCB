// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands.GalacticSearchCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.RetractIntakeCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchWithRetract extends SequentialCommandGroup {

  DriveTrainSubsystem m_driveTrainSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  LimelightSubsystem m_limelightSubsystem;

  /** Creates a new GalacticSearchWithRetract. */
  public GalacticSearchWithRetract(DriveTrainSubsystem dt, IntakeSubsystem intake, LimelightSubsystem ll) {
    m_driveTrainSubsystem = dt;
    m_intakeSubsystem = intake;
    m_limelightSubsystem = ll;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());



    addCommands(
      new GalacticSearchWithIntake(m_driveTrainSubsystem, m_intakeSubsystem, m_limelightSubsystem),
      new RetractIntakeCommand(m_intakeSubsystem)
    );
  }
}
