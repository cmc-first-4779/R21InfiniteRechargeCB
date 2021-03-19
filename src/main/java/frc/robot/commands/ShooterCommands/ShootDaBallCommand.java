// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  int buttonNumber;

  /** Creates a new ShootDaBall. */
  public ShootDaBallCommand(int buttonNumber, ShooterSubsystem shooter, ElevatorSubsystem elevator,
      HopperSubsystem hopper, LimelightSubsystem limelight) {
    this.shooter = shooter;
    this.elevator = elevator;
    this.hopper = hopper;
    this.limelight = limelight;
    this.buttonNumber = buttonNumber;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, elevator, hopper);
  }

  public ShootDaBallCommand(ShooterSubsystem shooter, ElevatorSubsystem elevator, HopperSubsystem hopper,
      LimelightSubsystem limelight) {
    new ShootDaBallCommand(0, shooter, elevator, hopper, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Commenting this out for now so that we can change our constant velocity based
    // off of our TY value
    // shooter.setConstantVelocity();

    // Get the defined velocity from our tY value
    int velocity = getVelocityfromSmartDashboard();
    // Pass that value to the shooter controllers
    shooter.setConstantVelocityFromInput(buttonNumber, velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isUpToSpeed()) {
      elevator.goUp(Constants.ELEVATOR_MOTOR_UP_SPEED);
      hopper.goBackward(Constants.HOPPER_MOTOR_BACKWARD_SPEED);
    } else {
      // elevator.stopElevator();
      // hopper.stop();
    }

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

  // Return the desired velocity of the shooter flyweel based on the Limelight tY
  // value
  public int getVelocityfromTY() {
    int velocity;
    double tY = limelight.getTY(); // Get the angle of the Limelight to the target
    // If we are in the "RED" zone, set the velocity to the Red zone velocity..
    if (tY <= Constants.SHOOTER_ANGLE_RED_ZONE) {
      //   Set the Vision Pipeline to use hardware zoom
      limelight.setPipeline(Constants.LIMELIGHT_PIPELINE_PORT_FAR);
      velocity = Constants.SHOOTER_VELOCITY_RED_ZONE;
    }
    // Else if we are in the "Blue" zone... (We also have to make sure we are NOT in
    // the RED zone)
    else if ((tY <= Constants.SHOOTER_ANGLE_BLUE_ZONE) && (tY > Constants.SHOOTER_ANGLE_RED_ZONE)) {
      //   Set the Vision Pipeline to use hardware zoom
      limelight.setPipeline(Constants.LIMELIGHT_PIPELINE_PORT_FAR);
      velocity = Constants.SHOOTER_VELOCITY_BLUE_ZONE;
    }
    // Else if we are in the "Yellow" zone... (We also have to make sure we are NOT
    // in the BLUE zone)
    else if ((tY <= Constants.SHOOTER_ANGLE_YELLOW_ZONE) && (tY > Constants.SHOOTER_ANGLE_BLUE_ZONE)) {
      //   Set the Vision Pipeline to use no zoom
      limelight.setPipeline(Constants.LIMELIGHT_PIPELINE_PORT_CLOSE);
      velocity = Constants.SHOOTER_VELOCITY_YELLOW_ZONE;
    }
    // Else.. We must be in the GREEN zone then..
    else {
       //   Set the Vision Pipeline to use no zoom
      limelight.setPipeline(Constants.LIMELIGHT_PIPELINE_PORT_CLOSE);     
      velocity = Constants.SHOOTER_VELOCITY_GREEN_ZONE;
    }
    // Return whatever we set the velocity to based on the angles above...
    return velocity;
  }

  private int getVelocityfromSmartDashboard() {
    int velocity = 0;

    switch (buttonNumber) {
      case 0:
        velocity = (int) SmartDashboard.getNumber("DefaultVelocity", Constants.SHOOTER_DEFAULT_VELOCITY);
        //   Set the Vision Pipeline to use no zoom
        limelight.setPipeline(Constants.LIMELIGHT_PIPELINE_PORT_CLOSE);
        break;
      case 1:
        //   Set the Vision Pipeline to use no zoom
        limelight.setPipeline(Constants.LIMELIGHT_PIPELINE_PORT_CLOSE);
        velocity = (int) SmartDashboard.getNumber("GreenZoneVelocity", Constants.SHOOTER_GREEN_ZONE_VELOCITY);
        break;
      case 2:
        velocity = (int) SmartDashboard.getNumber("RedZoneVelocity", Constants.SHOOTER_RED_ZONE_VELOCITY);
        //   Set the Vision Pipeline to use hardware zoom
        limelight.setPipeline(Constants.LIMELIGHT_PIPELINE_PORT_FAR);
        break;
      case 3:
        velocity = (int) SmartDashboard.getNumber("BlueZoneVelocity", Constants.SHOOTER_BLUE_ZONE_VELOCITY);
        //   Set the Vision Pipeline to use hardware zoom
        limelight.setPipeline(Constants.LIMELIGHT_PIPELINE_PORT_FAR);
        break;
      case 4:
        velocity = (int) SmartDashboard.getNumber("YellowZoneVelocity", Constants.SHOOTER_YELLOW_ZONE_VELOCITY);
        //   Set the Vision Pipeline to use no zoom
        limelight.setPipeline(Constants.LIMELIGHT_PIPELINE_PORT_CLOSE);
        break;
    }

    return velocity;
  }

}
