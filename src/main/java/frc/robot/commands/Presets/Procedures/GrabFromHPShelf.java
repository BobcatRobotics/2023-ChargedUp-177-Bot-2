// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets.Procedures;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.Presets.SetArm;
import frc.robot.commands.Presets.SetElevator;
import frc.robot.commands.Presets.SetWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabFromHPShelf extends SequentialCommandGroup {
  /** Creates a new GrabFromHPShelf. */
  public GrabFromHPShelf(Elevator e, Arm a, Wrist w) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevator(e, 0),
      new SetArm(a, ArmConstants.minNonCollidingExtention),
      new ParallelCommandGroup(new SetElevator(e, 3), new SetWrist(w, WristConstants.shelfPickupPos)),
      new SetArm(a, ArmConstants.shelfArmPos)
    );
  }
}
