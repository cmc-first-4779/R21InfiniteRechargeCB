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
import frc.robot.StaticConstants.BlingConstants;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


//   This command allows us to switch over control of the drivetrain from the 
//     driver to the operator.   Because the shooter is on the back of the robot
//     we will reverse the arcade drive X and Y values to make it easier for the 
//     drive team.
//   
//   NOTE:   This command will be executed by using a button on the driver's joystick
//
public class DriveWithJoystickReverseCommand extends CommandBase {
  /**
   * Creates a new DriveWithJoystickReverseCommand.
   */

     //Declare our local class variables
     DriveTrainSubsystem m_driveTrainSubsystem;
     XboxController m_joyStick;
     LimelightSubsystem m_limelightSubsystem;
     BlingSubsystem m_blingSubsystem;

  public DriveWithJoystickReverseCommand(DriveTrainSubsystem driveTrainSubsystem, XboxController joyStick, LimelightSubsystem limelightSubsystem, BlingSubsystem blingSubsystem) {

        //Pass our Subsystem and Joystick to our class
        m_driveTrainSubsystem = driveTrainSubsystem;
        m_joyStick = joyStick;
        m_limelightSubsystem = limelightSubsystem;
        m_blingSubsystem = blingSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrainSubsystem, m_limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //   Turn the Limelight to User Mode
    //m_limelightSubsystem.initLimelightforDriver();
    //   Let's Bling
    m_blingSubsystem.setBlingPattern(BlingConstants.BLING_PARTY_PALETTE);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Call our Arcade Drive Method in the DriveTrain Subsystem
    //Grabbing the Left-Hand joystic Y and X values using Hand.kLeft.
    // Negating traditional Y and x values to allow the robot to drive and aim backwards
    m_driveTrainSubsystem.arcadeDrive(m_joyStick.getY(Hand.kLeft) * Constants.DRIVETRAIN_JOYSTICK_Y_THROTTLE, (m_joyStick.getX(Hand.kLeft) * Constants.DRIVETRAIN_JOYSTICK_X_THROTTLE));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //  Turn Vision back on for the Limelight
    //m_limelightSubsystem.initLimelightforVision();
    //  Set the Bling back to default
    m_blingSubsystem.setBlingPattern(Constants.BLING_DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
