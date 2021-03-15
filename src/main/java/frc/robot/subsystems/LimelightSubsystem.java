/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.StaticConstants.LimelightConstants;;

public class LimelightSubsystem extends SubsystemBase {
  /**
   * Creates a new LimelightSubsystem.
   */

   //Declare the Network Table that our Limelight will broadcast its values on
   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
   //Declare the table entries for tx, ty, ta, and ts
   NetworkTableEntry tx = table.getEntry("tx");  //how far "x" axis is from our target
   NetworkTableEntry ty = table.getEntry("ty");  //how far "y" axis is from our target
   NetworkTableEntry ta = table.getEntry("ta");  //how much total area is the target taking up.  (how far away)
   NetworkTableEntry ts = table.getEntry("ts");  //How far are we skewed fromt the target
 
   //read values periodically
   double x = tx.getDouble(0.000);
   double y = ty.getDouble(0.000);
   double area = ta.getDouble(0.0);
   double skew = ts.getDouble(0.0); 

  //post to smart dashboard periodically
  //SmartDashboard.putNumber("LimelightX", x);
  //SmartDashboard.putNumber("LimelightY", y);
  //SmartDashboard.putNumber("LimelightArea", area);
  //SmartDashboard.putNumber("LimelightSkew", skew);

  public LimelightSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //SET THE LIMELIGHT CAMERA MODE
	//  0 = Limelight Vision.  VISION PROCESSING
	//  1 = Driver Camera (Increases exposure, disables vision processing)
  public void setCameraMode(double camMode) {
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(<value>);
    NetworkTableEntry testEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode");
    
    double before = testEntry.getDouble(0.0);
    System.out.println("Value of Table Entry before Command");
    System.out.println(before);
    
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(camMode);
    
    double after = testEntry.getDouble(0.0);
    System.out.println("Value of Table Entry after Command");
    System.out.println(after);
    System.out.println("**************************************");
    //System.out.println("Joystick Button was pressed.");
  }

  //  TURN THE LIMELIGHT LED LIGHTS ON AND OFF
	//  0 = use the LED Mode set in the current pipeline
	//  1 = Force the LEDs OFF
	//  2 = Force the LEDs to BLINK
	//  3 = Force the LEDs ON
  public void setLEDMode(double ledMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledMode);
  }

  //  SET WHICH VISION PIPELINE WE WANT TO USE..
  //    Pipelines:   0 - 9
  //    Each pipeline can be configured for a specific vision target
  public void setPipeline(double pipeline) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
  } 

  //  Return TX, which is the Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  public double getTX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  //  Returns TY, which is the Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  public double getTY() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  //  Returns TV, which is whether the limelight has any valid targets 
  //   0 = NO TARGET
  //   1 = TARGET
  public double getTV() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }

  //  Returns TA, which is the Target Area (0% of image to 100% of image)
  public double getTA() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }


  //  Returns TS, which is the Skew or rotation (-90 degrees to 0 degrees)
  public double getTS() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
  }

  //  Returns the approximate distance to the target..  
  //  NOTE:  AS PER LIMELIGHT, THIS IS NOT THE MOST OPTIMAL WAY TO ACQUIRE A TARGET.
  public double getDistance()  {
    //double distance = (RobotMap.LIMELIGHT_CARGO_HEIGHT - RobotMap.LIMELIGHT_CAMERA_HEIGHT) / Math.tan(getAY() - RobotMap.LIMELIGHT_CAMERAMOUNT_ANGLE);
    double distance =  4*Math.sqrt(getTA());
    return distance;
  }

  //  Returns whether or not the Limelight has a target...
  public boolean hasTarget() {
    double tv = getTV();
    if (tv == LimelightConstants.LIMELIGHT_NO_TARGET) {
      return false;
    }
    else {
      return true;
    }
  }

  public void setStreamingMode(double streamingMode){
    double m_streaming_mode = streamingMode;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(m_streaming_mode);
    setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_ON);
  }

  public boolean isHeadonTarget(){
  return getTS() == 0;
  }

  public boolean isRightOfTarget() {
    double ts = getTS();
  return ts <= Constants.LIMELIGHT_SKEW_CLOCKWISE_MAX && ts >= Constants.LIMELIGHT_SKEW_CLOCKWISE_MIN;
  }

  public boolean isLeftOfTarget() {
    double ts = getTS();
  return ts >= Constants.LIMELIGHT_SKEW_COUNTERCLOCKWISE_MAX && ts <= Constants.LIMELIGHT_SKEW_COUNTERCLOCKWISE_MIN;
  }

  public double skewDegrees() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry thor = table.getEntry("thor");
    NetworkTableEntry tvert = table.getEntry("tvert");
    double currentRatio = thor.getDouble(0.0) / tvert.getDouble(0.0);
    double originalRatio = 77.0 / 35.0;  // the largest possible ratio from the front
    double ratio = Math.min(1, currentRatio/originalRatio); // finding acos of a value > 1 will give NaN 
  return Math.toDegrees(Math.acos(ratio));
}

//  Return the target offset
public double returnTargetOffset()  {
    //declare PIDs for AIMing
    double kpAim = Constants.LIMELIGHT_AIMING_kpAim;
    double kpDist = Constants.LIMELIGHT_AIMING_kpDist; 
    double AimMinCmd = Constants.LIMELIGHT_AIMING_AIM_MIN_CMD;

    //Get the Target offsets from the NetworkTable
    double targetX = getTX();
    double targetY = getTY();
   

    //  Aim error and distance error based on calibrated Limelight crosshair
    double aimError = targetX;
    double distError = targetY;

    //Steering adjust with a degree deadband 
    double steeringAdjust = kpAim * aimError;
    
    //if our target is to the left of our crosshairs
    if (targetX > Constants.LIMELIGHT_AIMING_DEADBAND ){
      steeringAdjust = steeringAdjust + AimMinCmd;
    // else if our target is to the right of our crosshairs  
    }else if  (targetX < -Constants.LIMELIGHT_AIMING_DEADBAND ){
      steeringAdjust = steeringAdjust - AimMinCmd;
    }

    //Distance adjust, drive to the correct distance from the goal
    double targetOffset = kpDist * distError;
  
    return targetOffset;
  }
  
  //   Setup our Initial Configuration settings for our Limelight.   As the match goes
  //     on, these may change
  public void initLimelightforVision(){
    //   Set the green LED On
    setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_ON);;
    //   Set the Camera mode to "Vision" so that the Vision can use it
    setCameraMode(LimelightConstants.LIMELIGHT_CAMMODE_VISION);
    //   Set the Vision Pipeline to our default
    setPipeline(Constants.LIMELIGHT_PIPELINE_POWERCELLPORT);

  }

    //   Adjust the Limelight settings for the driver mode
  public void initLimelightforDriver(){
    //   Set the green LED Off
    setLEDMode(LimelightConstants.LIMELIGHT_LEDMODE_OFF);;
    //   Set the Camera mode to "Driver" so that the Driver can use it
    setCameraMode(LimelightConstants.LIMELIGHT_CAMMODE_DRIVER);
  }


}
