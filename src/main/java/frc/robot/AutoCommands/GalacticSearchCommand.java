// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.AutoCommands.GalacticSearchCommands.Path_A_BlueCommand;
import frc.robot.AutoCommands.GalacticSearchCommands.Path_A_RedCommand;
import frc.robot.AutoCommands.GalacticSearchCommands.Path_B_BlueCommand;
import frc.robot.AutoCommands.GalacticSearchCommands.Path_B_RedCommand;
import frc.robot.commands.DriveTrainCommands.DriveStraightCommand;
import frc.robot.commands.DriveTrainCommands.ResetDriveGyro;
import frc.robot.commands.DriveTrainCommands.TurnToAngleCommand;
import frc.robot.commands.DriveTrainCommands.TurnWithPIDCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchCommand extends SequentialCommandGroup {
  DriveTrainSubsystem m_driveTrainSubsystem;
  LimelightSubsystem m_limelightSubsystem;
  double tx;

  /** Creates a new GalacticSearch Command */
  public GalacticSearchCommand(DriveTrainSubsystem dt, LimelightSubsystem ll, RobotContainer rContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_driveTrainSubsystem = dt;
    m_limelightSubsystem = ll;

    rContainer.setTx(ll.getTX());

    if (m_limelightSubsystem.hasTarget()) {
     tx = m_limelightSubsystem.getTX();

      if (tx < Constants.GS_B3_MAX) {
        // Run RedBPath
        addCommands(new Path_B_RedCommand(m_driveTrainSubsystem));
      } else if (tx < Constants.GS_C3_MAX && tx > Constants.GS_C3_MIN) {
        // Run RedAPath
        addCommands(new Path_A_RedCommand(m_driveTrainSubsystem));
      } else if (tx < Constants.GS_D6_MAX && tx > Constants.GS_D6_MIN) {
        // Run BlueBPath
        addCommands(new Path_B_BlueCommand(m_driveTrainSubsystem));
      } else {
        // Run BlueAPath
        addCommands(new Path_A_BlueCommand(m_driveTrainSubsystem));
      }

    }

    
  
  
  }
  
}
