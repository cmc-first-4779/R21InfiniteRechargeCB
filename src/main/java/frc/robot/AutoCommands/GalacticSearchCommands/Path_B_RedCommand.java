// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands.GalacticSearchCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrainCommands.DriveStraightCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Path_B_RedCommand extends SequentialCommandGroup {
DriveTrainSubsystem m_driveTrainSubsystem;


  /** Creates a new RedBPathCommand. */
  public Path_B_RedCommand(DriveTrainSubsystem dt) {
    m_driveTrainSubsystem = dt;
    SmartDashboard.putString("BRED", "Got into BRed");
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveStraightCommand(m_driveTrainSubsystem, 20));
  }
}
