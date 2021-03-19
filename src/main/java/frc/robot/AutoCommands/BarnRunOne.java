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
public class BarnRunOne extends SequentialCommandGroup {
  /** Creates a new BarnRunOne. */

  DriveTrainSubsystem m_driveTrainSubsystem;
 
  
  public BarnRunOne(DriveTrainSubsystem dt) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    m_driveTrainSubsystem = dt;
    



    addCommands(

      new ResetDriveGyro(m_driveTrainSubsystem),
      new DriveStraightCommand(m_driveTrainSubsystem, 180),
      //new TimerCommand(3),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 90),
      new DriveStraightCommand(m_driveTrainSubsystem, 60),
      new TurnToAngleCommand(m_driveTrainSubsystem, 10, 225),
      new DriveStraightCommand(m_driveTrainSubsystem, 48)
      

    );
  }
}
