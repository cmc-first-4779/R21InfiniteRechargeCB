// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrainCommands.DriveStraightCommand;
import frc.robot.commands.DriveTrainCommands.ResetDriveGyro;
import frc.robot.commands.DriveTrainCommands.TurnToAngleCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Path_B_Drive extends SequentialCommandGroup {
  DriveTrainSubsystem m_driveTrainSubsystem;
  /** Creates a new Path_B_Drive. */
  public Path_B_Drive(DriveTrainSubsystem dt) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_driveTrainSubsystem = dt;
    addCommands(
      new ResetDriveGyro(m_driveTrainSubsystem),
      new DriveStraightCommand(m_driveTrainSubsystem, 240),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 45),
      new DriveStraightCommand(m_driveTrainSubsystem, 84.85),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 178),
      new DriveStraightCommand(m_driveTrainSubsystem, 170),
      new DriveStraightCommand(m_driveTrainSubsystem, -180)
    ); 
  }
}
