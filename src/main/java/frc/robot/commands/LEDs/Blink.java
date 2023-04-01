// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlinkinLEDs;

public class Blink extends CommandBase {
  /** Creates a new Blink. */
  BlinkinLEDs m_leds;
  boolean isPurple;
  Timer t;

  public Blink(BlinkinLEDs m_leds, boolean isPurple){
    this.m_leds = m_leds;
    this.isPurple = isPurple;
    t = new Timer();
    addRequirements(m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    t.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    t.start();

    if (!t.hasElapsed(1)){
      if(isPurple){
        m_leds.blinkPurple();
      }else{
        m_leds.blinkYellow();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(isPurple){
      m_leds.setPurple();
    }else{
      m_leds.setYellow();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return t.hasElapsed(1);
  }
}
