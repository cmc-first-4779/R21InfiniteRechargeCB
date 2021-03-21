// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands.GalacticSearchCommands;

import java.util.Timer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveTrainCommands.DriveStraightCommand;
import frc.robot.commands.DriveTrainCommands.ResetDriveGyro;
import frc.robot.commands.DriveTrainCommands.TurnToAngleCommand;
import frc.robot.commands.DriveTrainCommands.TurnWithPIDCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Path_B_BlueCommand extends SequentialCommandGroup {
DriveTrainSubsystem m_driveTrainSubsystem;


  /** Creates a new RedBPathCommand. */
  public Path_B_BlueCommand(DriveTrainSubsystem dt) {
    m_driveTrainSubsystem = dt;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ResetDriveGyro(m_driveTrainSubsystem),
    new TurnWithPIDCommand(m_driveTrainSubsystem, -3),
    new DriveStraightCommand(m_driveTrainSubsystem, 216),
    new TurnWithPIDCommand(m_driveTrainSubsystem, -27),
    // new WaitCommand(.5),
    new DriveStraightCommand(m_driveTrainSubsystem, -125),
    new TurnWithPIDCommand(m_driveTrainSubsystem, 2),
    new DriveStraightCommand(m_driveTrainSubsystem, 210));
    // new TurnWithPIDCommand(m_driveTrainSubsystem, 10),
    // new DriveStraightCommand(m_driveTrainSubsystem, 168),
    // new TurnWithPIDCommand(m_driveTrainSubsystem, -34),
    // new DriveStraightCommand(m_driveTrainSubsystem, 84),
    // new TurnWithPIDCommand(m_driveTrainSubsystem, 41),
    // new DriveStraightCommand(m_driveTrainSubsystem, 84));
    // new DriveStraightCommand(m_driveTrainSubsystem, 40));
  }
}
