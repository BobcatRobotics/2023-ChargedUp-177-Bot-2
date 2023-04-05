// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PWMLEDs;

public class BlinkPWM extends CommandBase {
  PWMLEDs leds;
  boolean isPurple;
  Timer t;

  /** Creates a new BlinkPWM. */
  public BlinkPWM(PWMLEDs leds, boolean isPurple) {
    this.leds = leds;
    this.isPurple = isPurple;
    t = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    t.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    t.start();

    if (!t.hasElapsed(1)) {
      if (isPurple) {
        leds.blinkPurple();
      } else {
        leds.blinkYellow();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (isPurple) {
      leds.setPurple();
    } else {
      leds.setYellow();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return t.hasElapsed(1);
  }
}
