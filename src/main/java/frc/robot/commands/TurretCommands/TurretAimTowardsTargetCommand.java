// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.StaticConstants.BlingConstants;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAimTowardsTargetCommand extends CommandBase {
  /** Creates a new AimTowardsTargetCommand. */

    /**
   * Creates a new AimTowardsTargetCommand.
   */
  LimelightSubsystem m_limelightSubsystem;
  TurretSubsystem m_turretSubsystem;
  BlingSubsystem m_blingSubsystem;
  double m_pipeline;

   //These will be used as we calculate turning 
  double turnPower;  //   Power needed to turn the robot
  boolean aimAndMove;
  double kpAim;  //  Our incremental power we use to hit the target
  double aimMinPower;  //  The minimum power we use to turn the robot to aim at the target

  boolean onTarget = false;  //  Are we on target, assume "false"
  int counter = 0;
  
  public TurretAimTowardsTargetCommand(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, double pipeline, BlingSubsystem blingSubsystem) {
            //Init our local variables
            m_turretSubsystem = turretSubsystem;
            m_limelightSubsystem = limelightSubsystem;
            m_pipeline = pipeline;
            m_blingSubsystem = blingSubsystem;
            kpAim = 0;
            aimMinPower = Constants.LIMELIGHT_SEEK_TURN_TURRET_POWER;
            aimAndMove = false;
            //  Put these to the Shuffleboard so that we can read them later to calibrate
            SmartDashboard.putNumber("kpAim", kpAim);
            SmartDashboard.putNumber("Aim Min", aimMinPower);
        
            // Use addRequirements() here to declare subsystem dependencies.
            addRequirements(m_turretSubsystem, m_limelightSubsystem, m_blingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        //  Set our "on target" boolean to false
        onTarget = false;
        //  set our counter to zero
        counter = 0;
        //  Set the Limelight to init for vision
        m_limelightSubsystem.initLimelightforVision();
        //  Set the bling subsystem to lime green
        m_blingSubsystem.setBlingPattern(BlingConstants.BLING_LIME);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
          /*
       * First check to see if there is a target, if not turn Then make sure we are on
       * target Then make sure we are the proper distance from target
       */
      boolean hasTarget = m_limelightSubsystem.hasTarget();
  
      //  If we have a target...
      if (hasTarget) {
        // Output to the RIOLOG whether the LimeLight has a target
        System.out.println("LimeLight Has Target");
        // Output to the Dashboard whether the Limelight has a target
        SmartDashboard.putString("LimeLight Has Target", "TARGET ACQUIRED");
        //  Calculate the amount of power we need to turn to line up to the target
        turnPower = calculateTurn();
        //  Output the amount of turn power  that we must turn to the RIOLOG
        System.out.println("Turn Power: " + turnPower);
        //  Output to the Dashboard the Turn Value that we must go to line up to the target
        SmartDashboard.putNumber("Turn Power", turnPower);

        
      } 
      // Else if we do NOT have a target, do not move
      else {
        // Output to the Dashboard whether the LimeLight has a target
        SmartDashboard.putString("LimeLight Has Target", "NO TARGET");  
        // Set the turn power equal to the minimum we use to seek    
        //turnPower = Constants.LIMELIGHT_SEEK_TURN_DT_POWER;
        turnPower = 0;
      }
      //  Send the turnpower into the Turret Motor..
      //   When this is negative it should turn the other way
      m_turretSubsystem.setTurretMotorSpeed(turnPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > 10;
  }

     /**
     * Calculates how much to turn. Get's how far off in the x-axis we are from limelight.  If we are within acceptable range, then it
     * returns 0.
     * @return the amount to turn
     */
    private double calculateTurn() {
      //  Find out how much error we have on the X-axis
      double aim_error = m_limelightSubsystem.getTX();
      //  GET the aimMin variable from the Dashboard so that we can calibrate.   We will comment this out later.
      aimMinPower = SmartDashboard.getNumber("Aim Min", aimMinPower);
      //  GET the kpAim variable from the Dashboard so that we can calibrate.   We will comment this out later.
      kpAim = SmartDashboard.getNumber("kpAim ", kpAim);
      //  If that error is too far to the right by our threshold / deadband, then increase 'turn' to turn left by our min constant.
      if (aim_error > Constants.LIMELIGHT_AIMING_DEADBAND) {
        //turn = (Constants.LIMELIGHT_AIMING_kpAim * aim_error) + Constants.LIMELIGHT_AIMING_AIM_MIN_CMD;
        turnPower = (kpAim * aim_error) + aimMinPower;
        onTarget = false;
        counter = 0;
      } 
      //  If that error is too far to the left by our threshold / deadband, then increase 'turn' to turn right by our min constant.
      else if (aim_error < -Constants.LIMELIGHT_AIMING_DEADBAND) {
        //turn = (Constants.LIMELIGHT_AIMING_kpAim * aim_error) - Constants.LIMELIGHT_AIMING_AIM_MIN_CMD;
        turnPower = (kpAim * aim_error) - aimMinPower;
        onTarget = false;
        counter = 0;
      } 
      //   If it is within the threshold / deadband, then DON'T TURN
      else {
        turnPower = 0;
        onTarget = true; 
        counter++;
      }
      //Return the directon and amount we have to turn
        return turnPower;
      }
}
