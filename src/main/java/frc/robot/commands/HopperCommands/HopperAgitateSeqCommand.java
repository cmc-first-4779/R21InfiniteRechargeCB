// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.HopperCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MiscCommands.TimerCommand;
import frc.robot.subsystems.HopperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HopperAgitateSeqCommand extends SequentialCommandGroup {
  /** Creates a new HopperAgitateSeqCommand. */

  //  This command will be a software agitate to try and stir up the balls in the hopper

  //  Declare our variables
  HopperSubsystem m_hopperSubsystem;

  public HopperAgitateSeqCommand(HopperSubsystem hopperSubsystem) {
    
    //  Init our variables
    m_hopperSubsystem = hopperSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new HopperGoBackwardCommand(m_hopperSubsystem),
    new TimerCommand(0.4)



    );
  }
}
