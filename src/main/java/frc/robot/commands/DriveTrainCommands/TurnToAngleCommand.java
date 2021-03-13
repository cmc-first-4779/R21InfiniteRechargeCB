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

public class TurnToAngleCommand extends CommandBase {
	private DriveTrainSubsystem m_driveTrainSubsystem;
	private double goalAngle = 0.0;
	private boolean isDone = false;
	private double speed;
	private double tolerance = 0.47;
	//private double currentAngle;
	
    public TurnToAngleCommand(DriveTrainSubsystem driveTrainSubsystem, double speed, double givenAngle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		m_driveTrainSubsystem = driveTrainSubsystem;
    	goalAngle = givenAngle;
    	this.speed = speed;
		isDone = false;    	
		SmartDashboard.putNumber("Goal Angle: ", goalAngle);
		SmartDashboard.putNumber("TurnSpeed: ", speed);
		addRequirements(m_driveTrainSubsystem);
    }

	// Called just before this Command runs the first time
	@Override
    public void initialize() {
    	//m_driveTrainSubsystem.resetGyro();
		isDone = false;

    }

	// Called repeatedly when this Command is scheduled to run
	@Override
    public void execute() {
    	double currentAngle = m_driveTrainSubsystem.getGyroAngle();
		 SmartDashboard.putNumber("Gyro: ", currentAngle);
		 //goalAngle = SmartDashboard.getNumber("Goal Angle: ", goalAngle);
	
		 speed = Constants.DRIVETRAIN_TURN_POWER;
    	if(Math.abs(goalAngle - currentAngle) < tolerance) {  //if within tolerance
    		m_driveTrainSubsystem.arcadeDrive(0.0, 0.0);
    		isDone = true;
    	} else if(currentAngle < goalAngle) {  //If left of target angle
    		m_driveTrainSubsystem.arcadeDrive(0, speed);  //turn clockwise
    	} else if(currentAngle > goalAngle){  //If right of target angle
    		m_driveTrainSubsystem.arcadeDrive(0, -speed);  //turn counterclockwise
    	}
    }

	// Make this return true when this Command no longer needs to run execute()
	@Override
    public boolean isFinished() {
		return isDone;
    }

    // Called once after isFinished returns true
    protected void end() {
		// Robot.drivetrain.setAHRSAdjustment(0.0);
		m_driveTrainSubsystem.arcadeDrive(0, 0);
		isDone = true;
    }
}
