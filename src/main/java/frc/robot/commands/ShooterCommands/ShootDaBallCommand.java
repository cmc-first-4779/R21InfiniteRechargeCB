// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootDaBallCommand extends CommandBase {
  ShooterSubsystem shooter;
  ElevatorSubsystem elevator; 
  HopperSubsystem hopper;


  /** Creates a new ShootDaBall. */
  public ShootDaBallCommand(ShooterSubsystem shooter, ElevatorSubsystem elevator, HopperSubsystem hopper) {
    this.shooter = shooter;
    this.elevator = elevator;
    this.hopper = hopper;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, elevator, hopper);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setConstantVelocity();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isUpToSpeed()) {
      elevator.goUp(Constants.ELEVATOR_MOTOR_UP_SPEED);
      hopper.goBackward(Constants.HOPPER_MOTOR_BACKWARD_SPEED);
    } else {
      elevator.stopElevator();
      hopper.stop();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
