// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class WristIO{
  
  @AutoLog
  public static class WristInputs{
    double wristCurrent = 0.0;
    boolean isAtCurrentLimit = false;
    double wristVoltage = 0.0;
    double wristTemperature = 0.0;
    double wristPosition = 0.0;
    double wristEncoderPosition = 0.0;
    double wristAbsolutePosition = 0.0;
    double wristVelocity = 0.0;
    double wristPercentOutput = 0.0;
    boolean wristUpperLimit = false;
    boolean wristLowerLimit = false;
    boolean wristIsAtSetpoint = false;
    double wristPositionError = 0.0;
    
  }
  public void updateInputs(WristInputsAutoLogged inputs){}
  public void resetToAbsolutePosition() {}
  public void setSpeed(double speed){}
  public void setSpeed0ArbitraryFeedForward(){}
  public void up(){}
  public void down(){}
  public void setState(double state){}
  public void setEncoderPosition(double position){}
  
}
