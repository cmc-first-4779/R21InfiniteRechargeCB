// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShooterCommands.ShootDaBallCommand;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretScanAndAimSeqCommand extends SequentialCommandGroup {
  TurretSubsystem m_turretSubsystem;
  LimelightSubsystem m_limelightSubsystem;
  double m_pipeline;
  BlingSubsystem m_blingSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  HopperSubsystem m_hopperSubsystem;

  /** Creates a new TurretScanAndAimSeqCommand. */
  public TurretScanAndAimSeqCommand(int buttonNumber, TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, double pipeline, BlingSubsystem blingSubsystem, ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem, HopperSubsystem hopperSubsystem) {
    
    m_turretSubsystem = turretSubsystem;
    m_limelightSubsystem = limelightSubsystem;
    m_pipeline = pipeline;
    m_blingSubsystem = blingSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_hopperSubsystem = hopperSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new TurretScanForTargetCommand(m_turretSubsystem, m_limelightSubsystem, m_pipeline, m_blingSubsystem),
      // new TurretAimTowardsTargetCommand(m_turretSubsystem, m_limelightSubsystem, m_pipeline, m_blingSubsystem),
      new ShootDaBallCommand( buttonNumber, m_shooterSubsystem, m_elevatorSubsystem, m_hopperSubsystem, m_limelightSubsystem)
      );

  }

  public TurretScanAndAimSeqCommand(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, double pipeline, BlingSubsystem blingSubsystem, ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem, HopperSubsystem hopperSubsystem) {
    new TurretScanAndAimSeqCommand(0, turretSubsystem, limelightSubsystem, pipeline, blingSubsystem, shooterSubsystem, elevatorSubsystem, hopperSubsystem);
  }
}
