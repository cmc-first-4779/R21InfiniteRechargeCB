/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveWithJoystickCommand extends CommandBase {
  /**
   * Creates a new DriveWithJoystickCommand.
   */

   //Declare our local class variables
  DriveTrainSubsystem m_driveTrainSubsystem;
  XboxController m_driverStick;

  public DriveWithJoystickCommand(DriveTrainSubsystem driveTrainSubsystem, XboxController driverStick) {
    //Pass our Subsystem and Joystick to our class
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_driverStick = driverStick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Call our Arcade Drive Method in the DriveTrain Subsystem
    //Grabbing the Right-Hand joystic Y and X values using Hand.kRight.
    // Negating the Y value since up on the Joystick is -1
    m_driveTrainSubsystem.arcadeDrive(-m_driverStick.getY(Hand.kLeft) * Constants.DRIVETRAIN_JOYSTICK_Y_THROTTLE, m_driverStick.getX(Hand.kLeft));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
