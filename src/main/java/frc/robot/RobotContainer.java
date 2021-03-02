// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.StaticConstants.XBoxJoystickConstants;
import frc.robot.commands.ElevatorCommands.GoDownCommand;
import frc.robot.commands.ElevatorCommands.GoUpCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommands.ShooterOffCommand;
import frc.robot.commands.ShooterCommands.ShooterOnCommand;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.TurretCommands.TurretMotorCounterClockwiseCommand;
import frc.robot.commands.TurretCommands.AimTowardsTargetCommand;
import frc.robot.commands.TurretCommands.TurretMotorClockwiseCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.HopperCommands.HopperGoBackwardCommand;
import frc.robot.commands.HopperCommands.HopperGoForwardCommand;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
   // The robot's subsystems and commands are defined here...
  
  //  Subsystems
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final ElevatorSubsystem ellivator = new ElevatorSubsystem();
  private final HopperSubsystem hopper = new HopperSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();

  //    Commands
  private final ShooterOnCommand shooterOn = new ShooterOnCommand(shooter);
  private final ShooterOffCommand shooterOff = new ShooterOffCommand(shooter);
  private final TurretMotorCounterClockwiseCommand turretMotorLeftCommand = new TurretMotorCounterClockwiseCommand(turret);
  private final TurretMotorClockwiseCommand turretMotorRightCommand = new TurretMotorClockwiseCommand(turret); 
  private final HopperGoForwardCommand hopperGoForward = new HopperGoForwardCommand(hopper);
  private final HopperGoBackwardCommand hopperGoBackward = new HopperGoBackwardCommand(hopper);
  private final GoDownCommand goback = new GoDownCommand(ellivator);
  private final GoUpCommand ellirun = new GoUpCommand(ellivator);
  private final AimTowardsTargetCommand aimTowardsTarget = new AimTowardsTargetCommand(turret, limelight, 1);
  private final TurretMotorClockwiseCommand turretMotorClockwiseCommand = new TurretMotorClockwiseCommand(turret);
  private final TurretMotorCounterClockwiseCommand turretMotorCounterClockwiseCommand = new TurretMotorCounterClockwiseCommand(turret);
  

  //  Joysticks
  private final XboxController driverStick = new XboxController(XBoxJoystickConstants.DRIVERSTICK_USB_PORT);
  private final XboxController operStick = new XboxController(XBoxJoystickConstants.OPERSTICK_USB_PORT); 
  
   
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(driverStick, 1).whenPressed(shooterOn);
    // new JoystickButton(driverStick, 2).whenPressed(shooterOff);
    // new JoystickButton(driverStick, 3).whileHeld(turretMotorRightCommand);
    // new JoystickButton(driverStick, 4).whileHeld(turretMotorLeftCommand);
    // new JoystickButton(driverStick, Button.kB.value).whileHeld(ellirun);
    // new JoystickButton(driverStick, Button.kX.value).whileHeld(goback);
    // new JoystickButton(driverStick, Button.kBumperLeft.value).whileHeld(hopperGoForward);
    // new JoystickButton(driverStick, Button.kBumperRight.value).whileHeld(hopperGoBackward);

    new JoystickButton(driverStick, Button.kA.value).whileHeld(aimTowardsTarget);
    new JoystickButton(driverStick, Button.kX.value).whileHeld(turretMotorClockwiseCommand);
    new JoystickButton(driverStick, Button.kY.value).whileHeld(turretMotorCounterClockwiseCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
    
