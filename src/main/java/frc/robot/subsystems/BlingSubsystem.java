/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BlingSubsystem extends SubsystemBase {

  //Declare our Bling Controller as a Spark.   (Same libraries from REV Robotics)
  Spark blingController;

  /**
   * Creates a new BlingSubsystem.
   */
  public BlingSubsystem() {
    //Initiate the blingController object
    blingController = new Spark(Constants.PWM_PORT_BLING);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //  Set the pattern for Bling to whatever pattern we pass it.
  //    *  All of our patterns are in the StaticConstants folder under BlingConstants
  public void setBlingPattern(double pattern){
    blingController.set(pattern);
  }
}
