// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
//import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.AutoCommands.HopperElevatorCommand;
import frc.robot.StaticConstants.XBoxJoystickConstants;
import frc.robot.commands.BlingCommands.BlingSetDefaultCommand;
import frc.robot.commands.DriveTrainCommands.DriveWithJoystickCommand;
import frc.robot.commands.ElevatorCommands.GoDownCommand;
import frc.robot.commands.ElevatorCommands.GoUpCommand;
import frc.robot.commands.ElevatorCommands.StopElevatorCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommands.ShootDaBallCommand;
import frc.robot.commands.ShooterCommands.ShooterOffCommand;
import frc.robot.commands.ShooterCommands.ShooterOnCommand;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.TurretCommands.TurretMotorCounterClockwiseCommand;
import frc.robot.commands.TurretCommands.TurretResetEncoderCommand;
import frc.robot.commands.TurretCommands.TurretScanForTargetCommand;
import frc.robot.commands.TurretCommands.StopTurretCommand;
import frc.robot.commands.TurretCommands.TurretAimTowardsTargetCommand;
import frc.robot.commands.TurretCommands.TurretMotorClockwiseCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.HopperCommands.HopperGoBackwardCommand;
import frc.robot.commands.HopperCommands.HopperGoForwardCommand;
import frc.robot.commands.HopperCommands.StopHopperCommand;
import frc.robot.commands.IntakeCommands.EjectCellsCommand;
import frc.robot.commands.IntakeCommands.ExtendIntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeCellsCommand;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;
import frc.robot.commands.IntakeCommands.RetractIntakeCommand;
import frc.robot.commands.LimelightCommands.LimelightInitForVisionCommand;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
   // The robot's subsystems and commands are defined here...
  
  //  Subsystems
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final BlingSubsystem m_blingSubsystem = new BlingSubsystem();
  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

//  Joysticks
  private final XboxController m_driverStick = new XboxController(XBoxJoystickConstants.DRIVERSTICK_USB_PORT);
  private final XboxController m_operStick = new XboxController(XBoxJoystickConstants.OPERSTICK_USB_PORT); 

  //    Commands
  private final ShooterOnCommand shooterOnCommand = new ShooterOnCommand(m_shooterSubsystem);
  private final ShooterOffCommand shooterOffCommand = new ShooterOffCommand(m_shooterSubsystem);
  private final ShootDaBallCommand shootDaBallCommand = new ShootDaBallCommand(m_shooterSubsystem, m_elevatorSubsystem, m_hopperSubsystem);
  private final TurretMotorCounterClockwiseCommand turretMotorLeftCommand = new TurretMotorCounterClockwiseCommand(m_turretSubsystem);
  private final TurretMotorClockwiseCommand turretMotorRightCommand = new TurretMotorClockwiseCommand(m_turretSubsystem); 
  private final StopTurretCommand turretStopCommand = new StopTurretCommand(m_turretSubsystem);
  private final HopperGoForwardCommand hopperGoForward = new HopperGoForwardCommand(m_hopperSubsystem);
  private final HopperGoBackwardCommand hopperGoBackward = new HopperGoBackwardCommand(m_hopperSubsystem);
  private final StopHopperCommand hopperStopCommand = new StopHopperCommand(m_hopperSubsystem);
  private final GoDownCommand elevatorGoDownCommand = new GoDownCommand(m_elevatorSubsystem);
  private final GoUpCommand elevatorGoUpCommand = new GoUpCommand(m_elevatorSubsystem);
  private final StopElevatorCommand elevatorStopCommand = new StopElevatorCommand(m_elevatorSubsystem);
  private final DriveWithJoystickCommand driveWithJoystickCommand = new DriveWithJoystickCommand(m_driveTrainSubsystem, m_driverStick);
  private final LimelightInitForVisionCommand limelightInitForVisionCommand = new LimelightInitForVisionCommand(m_limelightSubsystem);
  private final BlingSetDefaultCommand blingSetDefaultCommand = new BlingSetDefaultCommand(m_blingSubsystem);
  private final IntakeStopCommand intakeStopCommand = new IntakeStopCommand(m_intakeSubsystem); 
  private final IntakeCellsCommand intakeCellsCommand = new IntakeCellsCommand(m_intakeSubsystem);
  private final RetractIntakeCommand retractIntakeCommand = new RetractIntakeCommand(m_intakeSubsystem);
  
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Setup the CameraServer
    CameraServer.getInstance().startAutomaticCapture();

    //  DEFAULT COMMANDS FOR EACH SUBSYSTEM
    //This is where we make the DriveWithJoystick Command the default command for the DriveTrain
    //  Drivetrain Default:  Drive with Joystick   
    m_driveTrainSubsystem.setDefaultCommand(driveWithJoystickCommand);
    //  Limelight Default:  Limelight Init for Vision
    m_limelightSubsystem.setDefaultCommand(limelightInitForVisionCommand);
    //  Bling Default:  Bling with the Default Pattern
    m_blingSubsystem.setDefaultCommand(blingSetDefaultCommand);
    //  Shooter Default:  Shooter Off
    m_shooterSubsystem.setDefaultCommand(shooterOffCommand);
    //  Elevator Default:  Elevator Off
    m_elevatorSubsystem.setDefaultCommand(elevatorStopCommand);
    //  Hopper Default:  Hopper Off
    m_hopperSubsystem.setDefaultCommand(hopperStopCommand);
    //  Turret Default:  Turret Off
    m_turretSubsystem.setDefaultCommand(turretStopCommand);
    //  Intake Default:  Intake Off
    m_intakeSubsystem.setDefaultCommand(intakeStopCommand);


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

    //Shooting commands

    new JoystickButton(m_driverStick, Button.kA.value).whenPressed(shooterOnCommand);
    new JoystickButton(m_driverStick, Button.kB.value).whenPressed(shooterOffCommand);
    new JoystickButton(m_driverStick, Button.kY.value).whenPressed(shootDaBallCommand);


    // Intake Commands
    //new JoystickButton(m_driverStick, Button.kBumperRight.value).whenPressed(intakeCellsCommand);
    //new JoystickButton(m_driverStick, Button.kBumperLeft.value).whenPressed(retractIntakeCommand);

    //new JoystickButton(m_driverStick, Button.kX.value).whileHeld(new HopperElevatorCommand(m_hopperSubsystem, m_elevatorSubsystem,
    //    m_intakeSubsystem));

    //new JoystickButton(m_driverStick, Button.kBack.value).whileHeld(new EjectCellsCommand(m_intakeSubsystem));
    new JoystickButton(m_driverStick, Button.kStart.value).whileHeld(new TurretMotorClockwiseCommand(m_turretSubsystem));
    new JoystickButton(m_driverStick, Button.kBack.value).whileHeld(new TurretMotorCounterClockwiseCommand(m_turretSubsystem));
    new JoystickButton(m_driverStick, Button.kY.value).whenPressed(new TurretScanForTargetCommand(m_turretSubsystem, m_limelightSubsystem, 1, m_blingSubsystem));
    new JoystickButton(m_driverStick, Button.kA.value).whileHeld(new TurretAimTowardsTargetCommand(m_turretSubsystem, m_limelightSubsystem, 1, m_blingSubsystem));
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

  //  Return the Drivetrian Subsystem
  public DriveTrainSubsystem getDriveTrainSubsystem(){
    return m_driveTrainSubsystem;
  }

  //  Return the Intake Subsystem
  public IntakeSubsystem getIntakeSubsystem(){
    return m_intakeSubsystem;
  }

  //  Return the Bling Subsystem
  public BlingSubsystem getBlingSubsystem(){
    return m_blingSubsystem;
  }  

  //  Return the Elevator Subsystem
  public ElevatorSubsystem getElevatorSubsystem(){
    return m_elevatorSubsystem;
  }
  
  //  Return the Hopper Subsystem
  public HopperSubsystem getHopperSubsystem(){
    return m_hopperSubsystem;
  }
  
  //  Return the Limelight Subsystem
  public LimelightSubsystem getLimelightSubsystem(){
    return m_limelightSubsystem;
  }
  
  //  Return the Shooter Subsystem
  public ShooterSubsystem getShooterSubsystem(){
    return m_shooterSubsystem;
  }  

  //  Return the Shooter Subsystem
  public TurretSubsystem getTurretSubsystem(){
    return m_turretSubsystem;
  }  

}
    
