// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommands.ShooterOff;
import frc.robot.commands.ShooterCommands.ShooterOnCommand;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.TurretCommands.TurretMotorLeftCommand;
import frc.robot.commands.TurretCommands.TurretMotorRightCommand;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ShooterSubsystem shooter;
  private final ShooterOnCommand shooterOn;
  private final ShooterOff shooterOff;
  private final XboxController controller;

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    

  //  Declare Shooter Variables  
    ShooterSubsystem shooter;
    ShooterOnCommand shooterOn;
    ShooterOffCommand shooterOff;
  //  Declare Turret Variables  
    TurretSubsystem turret;
    TurretMotorLeftCommand turretMotorLeftCommand;
    TurretMotorRightCommand turretMotorRightCommand;
    
  //Declare XboxController  
    XboxController controller;
    
   
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
  //Init Turret Variables  
    turret = new TurretSubsystem();
    turretMotorLeftCommand = new TurretMotorLeftCommand(turret);
    turretMotorRightCommand = new TurretMotorRightCommand(turret);
    
  //Init Shooter Variables  
    shooter = new ShooterSubsystem();
    shooterOn = new ShooterOnCommand(shooter);
    shooterOff = new ShooterOff(shooter);
    
  //Init XBox Controller Variables  
    controller = new XboxController(0);

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
    new JoystickButton(controller, 1).whenPressed(shooterOn);
    new JoystickButton(controller, 2).whenPressed(shooterOff);
    new JoystickButton(controller, 3).whileHeld(turretMotorRightCommand);
    new JoystickButton(controller, 4).whileHeld(turretMotorLeftCommand);
    
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
