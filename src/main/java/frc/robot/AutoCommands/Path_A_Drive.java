// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrainCommands.DriveStraightCommand;
import frc.robot.commands.DriveTrainCommands.ResetDriveGyro;
import frc.robot.commands.DriveTrainCommands.TurnToAngleCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Path_A_Drive extends SequentialCommandGroup {
  DriveTrainSubsystem m_driveTrainSubsystem;
  /** Creates a new Path_A_Drive. */
  public Path_A_Drive(DriveTrainSubsystem dt) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_driveTrainSubsystem = dt;
    addCommands(
      new ResetDriveGyro(m_driveTrainSubsystem),
      new DriveStraightCommand(m_driveTrainSubsystem, 60), //5ft
      new TurnToAngleCommand(m_driveTrainSubsystem, 70, 90), 
      new DriveStraightCommand(m_driveTrainSubsystem, 30), //2.5ft
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 0),
      new DriveStraightCommand(m_driveTrainSubsystem, 60), //5ft
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 90),
      new DriveStraightCommand( m_driveTrainSubsystem, 30),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 0),
      new DriveStraightCommand(m_driveTrainSubsystem, 39),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, -90),
      new DriveStraightCommand(m_driveTrainSubsystem, 120),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 90),
      new DriveStraightCommand(m_driveTrainSubsystem, 30),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 0),
      new DriveStraightCommand(m_driveTrainSubsystem, 30),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 90),
      new DriveStraightCommand(m_driveTrainSubsystem, 30),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 0),
      new DriveStraightCommand(m_driveTrainSubsystem, 120),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 135),
      new DriveStraightCommand(m_driveTrainSubsystem, 96),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 180)
    );
  }
}
