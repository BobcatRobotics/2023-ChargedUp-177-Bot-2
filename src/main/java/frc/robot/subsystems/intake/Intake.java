// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.MathUtils;

public class Intake extends SubsystemBase {
  private final double cubeThreshold = 20.0;
  private final double coneThreshold = 20.0;

  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }
  
  public void runIntakeIn(){
    //motor.set(ControlMode.PercentOutput, -0.9);
    io.setIntakePercentOutput(-0.9);
  }

  public void runIntakeOut(){
    //motor.set(ControlMode.PercentOutput, 0.4);
    io.setIntakePercentOutput(0.4);
  } 

  public void runIntakeOutFull(){
    //motor.set(ControlMode.PercentOutput, 1);
    io.setIntakePercentOutput(1);
  }
  public void runIntakeInSlow(){
    //motor.set(ControlMode.PercentOutput, -0.06);
    io.setIntakePercentOutput(-0.06);
  }

  public boolean isAtCurrentLimit() {
    return inputs.intakeCurrent >= 20.0;
  }

  public boolean cubeSecured() {
    return inputs.intakeCurrent >= cubeThreshold;
  }

  public boolean coneSecured() {
    return inputs.intakeCurrent >= coneThreshold;
  }

  public double getCurrent() {
    return inputs.intakeCurrent;
  }

  public void runIntakePercent(double speed){
    speed = MathUtils.throttlePercent(speed);
    //motor.set(ControlMode.PercentOutput, speed);
    io.setIntakePercentOutput(speed);
  }

  public void stop(){
   //motor.set(ControlMode.PercentOutput, 0); 
    io.setIntakePercentOutput(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
  }
}
