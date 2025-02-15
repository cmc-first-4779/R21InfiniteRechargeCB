// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.AutoCommands.GalacticSearchCommands.GalacticSearchCommand;
import frc.robot.AutoCommands.GalacticSearchCommands.GalacticSearchWithIntake;
import frc.robot.AutoCommands.GalacticSearchCommands.GalacticSearchWithRetract;
import frc.robot.AutoCommands.GalacticSearchCommands.Path_A_BlueCommand;
import frc.robot.AutoCommands.GalacticSearchCommands.Path_A_RedCommand;
import frc.robot.AutoCommands.GalacticSearchCommands.Path_B_BlueCommand;
import frc.robot.AutoCommands.GalacticSearchCommands.Path_B_RedCommand;
import frc.robot.StaticConstants.XBoxJoystickConstants;
import frc.robot.commands.BlingCommands.BlingSetDefaultCommand;
import frc.robot.commands.DriveTrainCommands.DriveStopCommand;
import frc.robot.commands.DriveTrainCommands.DriveWithJoystickCommand;
import frc.robot.commands.DriveTrainCommands.ResetDriveGyro;
import frc.robot.commands.ElevatorCommands.StopElevatorCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommands.ShootDaBallCommand;
import frc.robot.commands.ShooterCommands.ShooterOffCommand;
import frc.robot.commands.ShooterCommands.ShooterOnCommand;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.TurretCommands.TurretScanAndAimSeqCommand;
import frc.robot.commands.TurretCommands.StopTurretCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.HopperCommands.StopHopperCommand;
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
  
  double tx; 
  
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
  //private final XboxController m_operStick = new XboxController(XBoxJoystickConstants.OPERSTICK_USB_PORT); 

  //    Commands
  private final ShooterOffCommand shooterOffCommand = new ShooterOffCommand(m_shooterSubsystem);
  private final StopTurretCommand turretStopCommand = new StopTurretCommand(m_turretSubsystem);
  private final StopHopperCommand hopperStopCommand = new StopHopperCommand(m_hopperSubsystem);
  private final StopElevatorCommand elevatorStopCommand = new StopElevatorCommand(m_elevatorSubsystem);
  private final DriveWithJoystickCommand driveWithJoystickCommand = new DriveWithJoystickCommand(m_driveTrainSubsystem, m_driverStick);
  private final LimelightInitForVisionCommand limelightInitForVisionCommand = new LimelightInitForVisionCommand(m_limelightSubsystem);
  private final BlingSetDefaultCommand blingSetDefaultCommand = new BlingSetDefaultCommand(m_blingSubsystem);
  private final IntakeStopCommand intakeStopCommand = new IntakeStopCommand(m_intakeSubsystem); 
  private final RetractIntakeCommand retractIntakeCommand = new RetractIntakeCommand(m_intakeSubsystem);
  
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Setup the CameraServer
    //CameraServer.getInstance().startAutomaticCapture();


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


    SmartDashboard.putData(m_driveTrainSubsystem);
    SmartDashboard.putData(m_turretSubsystem);

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

    //new JoystickButton(m_driverStick, Button.kA.value).whenPressed(new DriveStraightCommand(m_driveTrainSubsystem, 180));
    //new JoystickButton(m_driverStick, Button.kX.value).whenPressed(new TurnToAngleCommand(m_driveTrainSubsystem, 10, -45));
    //new JoystickButton(m_driverStick, Button.kB.value).whileHeld(new DriveStopCommand(m_driveTrainSubsystem));

    //new JoystickButton(m_driverStick, Button.kA.value).whenPressed(shooterOnCommand);
    new JoystickButton(m_driverStick, Button.kBumperLeft.value).whenPressed(shooterOffCommand);
    //new JoystickButton(m_driverStick, Button.kY.value).whenPressed(shootDaBallCommand);


    // Intake Commands

  new JoystickButton(m_driverStick, Button.kA.value).whenPressed(new IntakeCellsCommand(m_intakeSubsystem));
   // new JoystickButton(m_driverStick, Button.kB.value).whenPressed(new DriveStopCommand(m_driveTrainSubsystem));

    //new JoystickButton(m_driverStick, Button.kX.value).whileHeld(new HopperElevatorCommand(m_hopperSubsystem, m_elevatorSubsystem,
    //    m_intakeSubsystem));
    

    //  Turret Commands
    //new JoystickButton(m_driverStick, Button.kBack.value).whileHeld(new EjectCellsCommand(m_intakeSubsystem));
    //new JoystickButton(m_driverStick, Button.kStart.value).whileHeld(new TurretMotorClockwiseCommand(m_turretSubsystem));
    //new JoystickButton(m_driverStick, Button.kBack.value).whileHeld(new HopperGoForwardCommand(m_hopperSubsystem));
    //new JoystickButton(m_driverStick, Button.kY.value).whenPressed(new TurretScanForTargetCommand(m_turretSubsystem, m_limelightSubsystem, 0, m_blingSubsystem));
    //new JoystickButton(m_driverStick, Button.kA.value).whenPressed(new ShootDaBallCommand(Button.kA.value, m_shooterSubsystem, m_elevatorSubsystem, m_hopperSubsystem, m_limelightSubsystem));
    new JoystickButton(m_driverStick, Button.kB.value).whenPressed(new TurretScanAndAimSeqCommand(Button.kB.value, m_turretSubsystem, m_limelightSubsystem, Constants.LIMELIGHT_PIPELINE_PORT_FAR, m_blingSubsystem, m_shooterSubsystem, m_elevatorSubsystem, m_hopperSubsystem));
    //new JoystickButton(m_driverStick, Button.kX.value).whenPressed(new TurretScanAndAimSeqCommand(Button.kX.value, m_turretSubsystem, m_limelightSubsystem, Constants.LIMELIGHT_PIPELINE_PORT_FAR, m_blingSubsystem, m_shooterSubsystem, m_elevatorSubsystem, m_hopperSubsystem));
    //new JoystickButton(m_driverStick, Button.kY.value).whenPressed(new TurretScanAndAimSeqCommand(Button.kY.value, m_turretSubsystem, m_limelightSubsystem, Constants.LIMELIGHT_PIPELINE_PORT_CLOSE, m_blingSubsystem, m_shooterSubsystem, m_elevatorSubsystem, m_hopperSubsystem));
    //new JoystickButton(m_driverStick, Button.kY.value).whenPressed(new HopperElevatorCommand(m_hopperSubsystem, m_elevatorSubsystem, m_intakeSubsystem));

    // new JoystickButton(m_driverStick, Button.kA.value).whenPressed(new ShooterOnCommand(m_shooterSubsystem));
    // new JoystickButton(m_driverStick, Button.kB.value).whenPressed(new ShooterOffCommand(m_shooterSubsystem));

    // Auton Commands
    //new JoystickButton(m_driverStick, Button.kStart.value).whenPressed(new Path_B_Overall(m_driveTrainSubsystem, m_intakeSubsystem));
    // new JoystickButton(m_driverStick, Button.kA.value).whenPressed(new Path_A_Overall(m_driveTrainSubsystem, m_intakeSubsystem));
    // new JoystickButton(m_driverStick, Button.kY.value).whenPressed(new TurnWithPIDCommand(m_driveTrainSubsystem, -30));
    // new JoystickButton(m_driverStick, Button.kX.value).whenPressed(new ResetDriveGyro(m_driveTrainSubsystem));
    //new JoystickButton(m_driverStick, Button.kStart.value).whenPressed(new ExtendIntakeCommand(m_intakeSubsystem));

    //new JoystickButton(m_driverStick, Button.kBumperRight.value).whenPressed(new GalacticSearchWithRetract(m_driveTrainSubsystem, m_intakeSubsystem, m_limelightSubsystem));
    // new JoystickButton(m_driverStick, Button.kBumperLeft.value).whenPressed(new Path_B_RedCommand(m_driveTrainSubsystem));
    // new JoystickButton(m_driverStick, Button.kA.value).whenPressed(new Path_A_RedCommand(m_driveTrainSubsystem));
    // new JoystickButton(m_driverStick, Button.kBumperRight.value).whenPressed(new Path_A_BlueCommand(m_driveTrainSubsystem));
    // new JoystickButton(m_driverStick, Button.kX.value).whenPressed(new Path_B_BlueCommand(m_driveTrainSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new GalacticSearchWithRetract(m_driveTrainSubsystem, m_intakeSubsystem, m_limelightSubsystem);
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

  public double getTx() {
    return tx;
  }

  public void setTx(double tx) {
    this.tx = tx;
  }



}
    
