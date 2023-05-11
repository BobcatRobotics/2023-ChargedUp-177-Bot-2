// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

/** Add your docs here. */
public class WristIOTalonFX extends WristIO{
    WPI_TalonFX motor;
    CANCoder wristEncoder;
    CANCoderConfiguration encoderConfig;
    PIDController pid;
    
    public WristIOTalonFX() {
        motor  = new WPI_TalonFX(Constants.wristMotorID);
        wristEncoder = new CANCoder(Constants.wristCanCoderID);
        wristEncoder.configFactoryDefault();
        //wristEncoder.configFeedbackCoefficient(4096/360, "falcons", SensorTimeBase.Per100Ms_Legacy);
        encoderConfig = new CANCoderConfiguration();
        wristEncoder.configAllSettings(encoderConfig);
        wristEncoder.configMagnetOffset(-WristConstants.wristOffset);
        wristEncoder.setPositionToAbsolute();
        motor.configRemoteFeedbackFilter(wristEncoder.getDeviceID(), RemoteSensorSource.CANCoder, 0);
        motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        motor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        motor.config_kF(0, 0);
        motor.config_kP(0, 0);
        motor.config_kI(0, 0);
        motor.config_kD(0, 0);
        motor.configPeakOutputForward(0.63, 20);
        motor.configPeakOutputReverse(-0.36, 20);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

        pid = new PIDController(-0.075, 0.0, 0.0); // increased from -.07 to compensate for loose chain
        pid.setTolerance(2); // increased from 2 to compensate for loose chain
        Timer.delay(1);
  }

  @Override
  public void updateInputs(WristInputsAutoLogged inputs) {
    inputs.wristCurrent = motor.getStatorCurrent();
    inputs.isAtCurrentLimit = motor.getStatorCurrent() >= 25;
    inputs.wristVoltage = motor.getMotorOutputVoltage();
    inputs.wristTemperature = motor.getTemperature();
    inputs.wristPosition = motor.getSelectedSensorPosition();
    inputs.wristEncoderPosition = wristEncoder.getPosition();
    inputs.wristAbsolutePosition = wristEncoder.getAbsolutePosition();
    inputs.wristVelocity = motor.getSelectedSensorVelocity();
    inputs.wristPercentOutput = motor.getMotorOutputPercent();
    inputs.wristUpperLimit =  wristEncoder.getPosition() <= WristConstants.globalWristMaxAngleUp;
    inputs.wristLowerLimit = wristEncoder.getPosition() >= WristConstants.globalWristMaxAngleDown;
    inputs.wristIsAtSetpoint = pid.atSetpoint();
    inputs.wristPositionError = motor.getClosedLoopError();
  }

  @Override
  public void resetToAbsolutePosition() {
    motor.setSelectedSensorPosition(wristEncoder.getAbsolutePosition());
  }

  @Override
  public void setSpeed(double speed){
    motor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void setSpeed0ArbitraryFeedForward(){
    motor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0.05);
  }

  @Override
  public void up(){
    motor.set(ControlMode.PercentOutput, 0.5);
  }

  @Override
  public void down(){
    motor.set(ControlMode.PercentOutput, -0.5);
  }

  @Override
  public void setState(double state) {
    motor.set(ControlMode.PercentOutput, pid.calculate(wristEncoder.getPosition(), state));
  }

  @Override
  public void setEncoderPosition(double position) {
    wristEncoder.setPosition(position);
  }
  
}
