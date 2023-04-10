// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeOutFullSpeed;
import frc.robot.commands.MountChargeStationInverse;
import frc.robot.commands.MountChargeStationInverseScore;
import frc.robot.commands.Presets.StartingConfig;
import frc.robot.commands.Presets.Procedures.GrabFromHPChute;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MountAndBalanceInverseScore extends SequentialCommandGroup {
  /** Creates a new MountAndBalanceInverse. */
  private Swerve s_Swerve;
  private Elevator elevator;
  private Arm arm;
  private Wrist wrist;
  private Intake intake;

  public MountAndBalanceInverseScore(Swerve s_Swerve, Elevator e, Arm a, Wrist w, Intake i) {
    this.s_Swerve = s_Swerve;
    elevator = e;
    wrist = w;
    arm = a;
    intake = i;

    addRequirements(s_Swerve, e, a, i, w);
    addCommands(
      new MountChargeStationInverseScore(s_Swerve),
      new SequentialCommandGroup(
            new GrabFromHPChute(elevator, arm, wrist),
            new IntakeOutFullSpeed(intake),
            new StartingConfig(elevator, arm, wrist)
      ),
      new BalanceChargeStation(s_Swerve, true, true)
    );
  }
}
