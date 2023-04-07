// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class SetWrist extends CommandBase {
  private Wrist w;
  private double pos;
  private Timer timer;
  private boolean atSetpoint = false;
  private boolean prevAtSetpoint = false;

  /** Creates a new SetWrist. */
  public SetWrist(Wrist w, double pos) {
    this.w = w;
    this.pos = pos;
    this.timer = new Timer();
    this.atSetpoint = false;
    this.prevAtSetpoint = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(w);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    w.setState(pos);
    //SmartDashboard.putNumber("while command running, wrist pid error", w.getClosedLoopError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SmartDashboard.putBoolean("wrist interrupted", interrupted);
    //SmartDashboard.putNumber("wrist pid error",w.getClosedLoopError());
    w.setSpeed0ArbitraryFeedForward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (w.isAtSetpoint()) {
    //   prevAtSetpoint = atSetpoint;
    //   atSetpoint = true;
    // } else {
    //   prevAtSetpoint = atSetpoint;
    //   atSetpoint = false;
    //   timer.stop();
    //   timer.reset();
    //   return false;
    // }

    // if (atSetpoint && !prevAtSetpoint) {
    //   timer.reset();
    //   timer.start();
    // }

    // if (timer.hasElapsed(0.05) && w.isAtSetpoint()) {
    //   return true;
    // }
    // return false;
    return w.isAtSetpoint();
  }
}
