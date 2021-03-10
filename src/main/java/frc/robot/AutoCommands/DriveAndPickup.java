// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.DriveTrainCommands.DriveStraightCommand;
import frc.robot.commands.IntakeCommands.IntakeCellsCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndPickup extends ParallelRaceGroup {
  DriveTrainSubsystem m_DriveTrainSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  /** Creates a new DriveAndPickup. */
  public DriveAndPickup(DriveTrainSubsystem dt,IntakeSubsystem intake) {
    m_DriveTrainSubsystem = dt;
    m_IntakeSubsystem = intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeCellsCommand(m_IntakeSubsystem),
      new DriveStraightCommand(m_DriveTrainSubsystem, 180)
    );
  }
}
