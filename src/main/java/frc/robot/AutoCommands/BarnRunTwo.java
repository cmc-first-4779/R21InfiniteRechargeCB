// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.IntakeCellsCommand;
import frc.robot.commands.IntakeCommands.RetractIntakeCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BarnRunTwo extends SequentialCommandGroup {

DriveTrainSubsystem m_driveTrainSubsystem;
IntakeSubsystem m_IntakeSubsystem;

  /** Creates a new BarnRunTwo. */
  public BarnRunTwo(IntakeSubsystem intake, DriveTrainSubsystem dt) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_IntakeSubsystem = intake;
    m_driveTrainSubsystem = dt;
    addCommands(
      new DriveAndPickup(m_driveTrainSubsystem, m_IntakeSubsystem),
      new RetractIntakeCommand(m_IntakeSubsystem)

    );
  }
}
