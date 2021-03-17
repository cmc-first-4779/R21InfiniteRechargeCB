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

public class TurnWithPIDCommand extends CommandBase {
	private DriveTrainSubsystem m_driveTrainSubsystem;
	private double goalAngle = 0.0;
	private boolean isDone = false;
	private double speed;
	private double tolerance = Constants.DRIVETRAIN_TURN_THRESHOLD;
	private double kPTurn = Constants.DRIVETRAIN_TURN_KP;
	int counter = 0;
	//private double currentAngle;
	
    public TurnWithPIDCommand(DriveTrainSubsystem driveTrainSubsystem, double givenAngle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		m_driveTrainSubsystem = driveTrainSubsystem;
    	goalAngle = m_driveTrainSubsystem.getGyroAngle() +  givenAngle;
    	this.speed = Constants.DRIVETRAIN_TURN_POWER;
		isDone = false;    	
		addRequirements(m_driveTrainSubsystem);
    }

	// Called just before this Command runs the first time
	@Override
    public void initialize() {
    	//m_driveTrainSubsystem.resetGyro();
		isDone = false;
		counter = 0;

    }

	// Called repeatedly when this Command is scheduled to run
	@Override
    public void execute() {
    	double currentAngle = m_driveTrainSubsystem.getGyroAngle();
		 SmartDashboard.putNumber("Gyro: ", currentAngle);
		 //goalAngle = SmartDashboard.getNumber("Goal Angle: ", goalAngle);
	
		 speed = calculateTurnSpeed(currentAngle);
    	if(Math.abs(goalAngle - currentAngle) < tolerance) {  //if within tolerance
			m_driveTrainSubsystem.arcadeDrive(0.0, 0.0);
    		counter++;
		} else if(currentAngle < goalAngle) {  //If left of target angle
			counter = 0;
    		m_driveTrainSubsystem.arcadeDrive(0, speed);  //turn clockwise
		} else if(currentAngle > goalAngle){  //If right of target angle
			counter = 0;
    		m_driveTrainSubsystem.arcadeDrive(0, -speed);  //turn counterclockwise
		}
		
		isDone = counter > 10;
    }

	private double calculateTurnSpeed(double currentAngle) {
		double turnError = Math.abs(currentAngle - goalAngle);
		speed = Constants.DRIVETRAIN_TURN_POWER + (kPTurn * turnError);
		SmartDashboard.putNumber("TurnError", turnError);
		SmartDashboard.putNumber("Speed", speed);
		return speed;
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
