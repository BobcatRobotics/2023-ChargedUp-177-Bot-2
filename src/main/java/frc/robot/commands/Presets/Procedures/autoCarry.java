// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets.Procedures;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Presets.SetArm;
import frc.robot.commands.Presets.SetWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoCarry extends SequentialCommandGroup {
  /** Creates a new autoCarry. */
  public autoCarry(Wrist w, Arm a) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArm(a, Constants.ArmConstants.autoCarry),
      new SetWrist(w, Constants.WristConstants.autoCarry)
    );
  }
}
