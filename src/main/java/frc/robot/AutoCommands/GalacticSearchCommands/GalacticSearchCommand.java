// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands.GalacticSearchCommands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchCommand extends SequentialCommandGroup {
  double tx;
  Supplier<Object> supplier = () -> "";
  Supplier<PathSelector> selector = () -> PathSelector.ARED;

  private enum PathSelector {
    ARED,
    ABLUE,
    BRED,
    BBLUE
  }

  private PathSelector select() {
    return selector.get();
  } 

  /** Creates a new GalacticSearch Command */
  public GalacticSearchCommand(DriveTrainSubsystem dt, LimelightSubsystem ll) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new InstantCommand(() -> {
      final String path;

      if (RobotBase.isSimulation()) {
        path = "Path_A_Red";
      } else {
        // Get tx
        tx = ll.getTX();
        SmartDashboard.putNumber("Galactic TX", tx);

        if (tx < Constants.GS_C3_MAX && tx > Constants.GS_C3_MIN) {
          path = "Path_A_Red";
          selector = () -> PathSelector.ARED;
        } else if (tx < Constants.GS_B3_MAX) {
          path = "Path_B_Red";
          selector = () -> PathSelector.BRED;
        } else if (tx < Constants.GS_D6_MAX && tx > Constants.GS_D6_MIN) {
          path = "Path_B_Blue";
          selector = () -> PathSelector.BBLUE;
        } else {
          path = "Path_A_BLUE";
          selector = () -> PathSelector.ABLUE;
        }
      }
      SmartDashboard.putString("GalacticSearch", path);
      supplier = () -> path;
    }), 
    // new SelectCommand(Map.ofEntries(Map.entry("Path_A_Red", new Path_A_RedCommand(dt)),
    //     Map.entry("Path_B_Red", new Path_B_RedCommand(dt)), Map.entry("Path_B_Blue", new Path_B_BlueCommand(dt)),
    //     Map.entry("Path_A_Blue", new Path_A_BlueCommand(dt))), supplier),
        new SelectCommand(Map.ofEntries(
          Map.entry(PathSelector.ARED, new Path_A_RedCommand(dt)),
          Map.entry(PathSelector.BRED, new Path_B_RedCommand(dt))
          ), this::select) 
        ,new PrintCommand("After Seclect Command"));

  }

}
