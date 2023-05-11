// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  WristIOTalonFX io;
  WristInputsAutoLogged inputs = new WristInputsAutoLogged();

  public Wrist(WristIOTalonFX io) {
    this.io = io;  
  }

  public void resetToAbsolutePosition() {
    io.resetToAbsolutePosition();
  }

  public void setSpeed(double speed){
    io.setSpeed(speed);
  }
  
  public void setSpeed0ArbitraryFeedForward(){
    io.setSpeed0ArbitraryFeedForward();
  }

  public double getClosedLoopError() {
    return inputs.wristPositionError;
  }

  public void up() {
    io.up();
  }

  public void down() {
    io.down();
  }

  public boolean isAtCurrentLimit() {
    return inputs.isAtCurrentLimit;
  }

  public double getAbsEncoderPos() {
    return inputs.wristAbsolutePosition;
  }

  public double getEncoderPos() {
    return inputs.wristEncoderPosition;
  }

  public double getPosition() {
    return inputs.wristPosition;
  }

  public boolean globalWristMaxAngleUp() {
    return inputs.wristUpperLimit;
  }

  public boolean globalWristMaxAngleDown() {
    return inputs.wristLowerLimit; // Second part is to make it work if it loops back around to 0
  }

  public void setState(double state) {
    io.setState(state);
  }

  public boolean isAtSetpoint() {
    return inputs.wristIsAtSetpoint;
    //return motor.isMotionProfileFinished();
  }

  // public void turnWrist(double speed){
  //   speed = MathUtils.throttlePercent(speed);
  //   //if lower limit switch is tripped and we're trying to go down, don't
  //   //if upper limit switch is tripped and we're trying to go up, don't
  //   //otherwise drive at given speed
  //   if(!((Math.signum(speed) == -1 && lowerLimit.get() == true))){
  //     if(!((Math.signum(speed) == 1 && lowerLimit.get() == false))){
  //       motor.set(TalonFXControlMode.PercentOutput, speed);
  //     }
  //   }
  // }
  @Override
  public void periodic() {
    // compressor.enableAnalog(80, 115);//TODO: check limits
    // SmartDashboard.putNumber("compressor psi", compressor.getPressure());
    //motor.setSelectedSensorPosition(getEncoderPos());
    if (inputs.wristPosition <= 5 && inputs.wristPosition >= 0) {
      io.setEncoderPosition(360);
    }
    //SmartDashboard.putNumber("wrist current", motor.getStatorCurrent());
    if (isAtCurrentLimit()) {
      io.setSpeed(0);
    }
    //SmartDashboard.putNumber("wrist absolute", getAbsEncoderPos());
    //SmartDashboard.putNumber("wrist pos", getPosition());
    //SmartDashboard.putNumber("wrist encoder", getEncoderPos());
    io.updateInputs(inputs);
  }
}
