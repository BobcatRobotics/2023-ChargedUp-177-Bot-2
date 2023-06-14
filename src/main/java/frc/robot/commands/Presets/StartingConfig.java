// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StartingConfig extends SequentialCommandGroup {
  /** Creates a new StartingConfig. */

  public StartingConfig(Elevator e, Arm a, Wrist w) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // if (e.getBottomLimits()) {
    //   addCommands(
    //     Commands.parallel(
    //       new SetWrist(w, WristConstants.startingPosWrist),
    //       new SetArm(a, ArmConstants.startingArmPos)
    //     )
    //   );
    // } else {
      addCommands(
        Commands.parallel(new SetArm(a, ArmConstants.minNonCollidingExtention), new SetWrist(w, WristConstants.startingPosWrist)),
        new SetElevator(e, 0, w),
        new SetArm(a, ArmConstants.startingArmPos)
      );
    // }
  }
}
