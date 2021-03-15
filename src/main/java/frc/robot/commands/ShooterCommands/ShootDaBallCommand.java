// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootDaBallCommand extends CommandBase {
  ShooterSubsystem shooter;
  ElevatorSubsystem elevator; 
  HopperSubsystem hopper;
  LimelightSubsystem limelight;


  /** Creates a new ShootDaBall. */
  public ShootDaBallCommand(ShooterSubsystem shooter, ElevatorSubsystem elevator, HopperSubsystem hopper, LimelightSubsystem limelight) {
    this.shooter = shooter;
    this.elevator = elevator;
    this.hopper = hopper;
    this.limelight = limelight;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, elevator, hopper);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //   Commenting this out for now so that we can change our constant velocity based off of our TY value
    //shooter.setConstantVelocity();

    // Get the defined velocity from our tY value
    int velocity = getVelocityfromTY();
    // Pass that value to the shooter controllers
    shooter.setConstantVelocityFromInput(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isUpToSpeed()) {
      elevator.goUp(Constants.ELEVATOR_MOTOR_UP_SPEED);
      hopper.goBackward(Constants.HOPPER_MOTOR_BACKWARD_SPEED);
    } else {
      //elevator.stopElevator();
      //hopper.stop();
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

  //  Return the desired velocity of the shooter flyweel based on the Limelight tY value
  public int getVelocityfromTY(){
    int velocity;
    double tY = limelight.getTY();  //Get the angle of the Limelight to the target
    //  If we are in the "RED" zone, set the velocity to the Red zone velocity..
    if(tY <= Constants.SHOOTER_ANGLE_RED_ZONE){
      velocity = Constants.SHOOTER_VELOCITY_RED_ZONE;
    }
    //  Else if we are in the "Blue" zone...  (We also have to make sure we are NOT in the RED zone)
    else if ((tY <= Constants.SHOOTER_ANGLE_BLUE_ZONE) && (tY > Constants.SHOOTER_ANGLE_RED_ZONE)){
      velocity = Constants.SHOOTER_VELOCITY_BLUE_ZONE;
    }
    //  Else if we are in the "Yellow" zone... (We also have to make sure we are NOT in the BLUE zone)
    else if ((tY <= Constants.SHOOTER_ANGLE_YELLOW_ZONE) && (tY > Constants.SHOOTER_ANGLE_BLUE_ZONE)){
      velocity = Constants.SHOOTER_VELOCITY_YELLOW_ZONE;
    }
    //  Else..  We must be in the GREEN zone then..
    else{
      velocity = Constants.SHOOTER_VELOCITY_GREEN_ZONE;
    }
    //  Return whatever we set the velocity to based on the angles above...
    return velocity;
  }

}
