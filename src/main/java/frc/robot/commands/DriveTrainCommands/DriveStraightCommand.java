/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveStraightCommand extends CommandBase {
  DriveTrainSubsystem m_driveTrain;
  double m_distance; // Position in sensor units
  int counter = 0;  //  Set counter to zero
  boolean firstTime = true; // Flag for knowing if this is first time running again.

  /**
   * Creates a new DriveStraightCommand.
   */
  public DriveStraightCommand(DriveTrainSubsystem drive_train, double distance) {
    m_driveTrain = drive_train;
    // Need to calculate the distance. Need to know ticks per rotation, gear
    // reduction, distance per rotation of wheel.
    double distancPerWheelRotation = Constants.DRIVETRAIN_WHEEL_DIAMETER * Math.PI;
    double multiplier = Constants.DRIVETRAIN_GEAR_REDUCTION_RATIO * Constants.DRIVETRAIN_ENCODER_TICKS_PER_ROTATION
        / distancPerWheelRotation;

    m_distance = distance * multiplier;
    SmartDashboard.putNumber("MM Distance", distance);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Zero out our Encoders
    m_driveTrain.zeroEncoders();

    m_driveTrain.simpleMM(m_distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (firstTime) {
      m_driveTrain.zeroEncoders();
      m_driveTrain.setMotorSafety(false);
      firstTime = false;
    }

    if (m_driveTrain.motionMagicOnTargetDrive(m_distance)) {
      System.out.println("on target " + counter);
      counter++;
    } else {
      counter = 0;
      // System.out.println("MM not on target");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0, 0);
    firstTime = true;
    m_driveTrain.setMotorSafety(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= Constants.DRIVETRAIN_STRAIGHT_COUNTER;
  }
}
