// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class WaitUntilTime extends CommandBase {
  /** Creates a new WaitUntilTime. */
  private double t;
  private Command c;

  /**
   * Waits until a specified time to occur. Upon that time has passed then run execute loop of command
   *
   * @param time time to wait for to execute command
   * @param c Command to run upon time being reached. This must be an endless command. IE a command that doesn't have a hard ending state (such as intake oult)
   */
  public WaitUntilTime(double time, Command c) {
    this.t = time;
    this.c = c;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.c.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.getMatchTime() >= t;
  }
}
