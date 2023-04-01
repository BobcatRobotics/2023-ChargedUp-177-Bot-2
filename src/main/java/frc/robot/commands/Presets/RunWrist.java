// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Wrist;

public class RunWrist extends CommandBase {
  /** Creates a new RunWrist. */
  private Wrist w;
  private JoystickButton rightTrigger;
  private JoystickButton rightBumper;

  public RunWrist(Wrist w, JoystickButton rightTrigger, JoystickButton rightBumper) {
    this.w = w;
    this.rightTrigger = rightTrigger;
    this.rightBumper = rightBumper;
    addRequirements(w);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // rightTrigger.whileTrue(new InstantCommand(w::down));
    // rightBumper.whileTrue(new InstantCommand(w::up));
    // if (!rightTrigger.getAsBoolean() && !rightBumper.getAsBoolean()) {
    //   w.setSpeed(0);
    // }
    if (w.globalWristMaxAngleUp() && rightBumper.getAsBoolean()) {
      w.setSpeed(0);
    } else if (w.globalWristMaxAngleDown() && rightTrigger.getAsBoolean()) {
      w.setSpeed(0);
    } else if (rightTrigger.getAsBoolean()) {
      w.down();
    } else if (rightBumper.getAsBoolean()) {
      w.up();
    } else {
      //w.setSpeed(0);
      w.setSpeed0ArbitraryFeedForward();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
