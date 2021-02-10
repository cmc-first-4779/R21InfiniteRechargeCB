/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.MiscCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//  This Command is used to put wait timers in our Command groups and wherever we need
//   to slow things down on the robot

public class TimerCommand extends CommandBase {
  /**
   * Creates a new TimerCommand.
   */

  Timer timer = new Timer();
	private double m_time;

  public TimerCommand(double time) {
    m_time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();  //Reset the timer.
    timer.start();  //Start the timer.
    //   Put the timer length to the dashoard
    SmartDashboard.putNumber("Timer Length", m_time);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Put the timer value to the Dashboard
    SmartDashboard.putNumber("Timer Value", timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();  //Stop the timer
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      	//  If our timer is less than the time we want it to run, keep running the command.
        if (timer.get() < m_time) {
          return false;
        }
        //  Else once we hit our alloted time, exit out of the command.
        else
          return true; 
  }
}
