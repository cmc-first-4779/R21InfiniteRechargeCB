// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ElevatorCommands.GoUpCommand;
import frc.robot.commands.HopperCommands.HopperGoBackwardCommand;
import frc.robot.commands.HopperCommands.HopperGoForwardCommand;
import frc.robot.commands.IntakeCommands.IntakeCellsCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HopperElevatorCommand extends ParallelCommandGroup {

  HopperSubsystem m_hopperSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  IntakeSubsystem m_intakeSubsystem;

  /** Creates a new HopperElevatorCommand. */
  public HopperElevatorCommand(HopperSubsystem hopperSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    super();

    m_hopperSubsystem = hopperSubsystem;
    m_elevatorSubsystem = elevatorSubsystem; 
    m_intakeSubsystem = intakeSubsystem;

    addCommands(
    new HopperGoBackwardCommand(m_hopperSubsystem),
    // new IntakeCellsCommand(m_intakeSubsystem),
    new GoUpCommand(m_elevatorSubsystem)
    );
  }
}
