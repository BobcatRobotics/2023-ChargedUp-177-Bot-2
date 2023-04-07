// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.Util.MathUtils;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private WPI_TalonFX motor;
  private CANCoder wristEncoder;
  private CANCoderConfiguration encoderConfig;

  private PIDController pid;
  // Solenoid solenoid;
  //double pressure;
  // PneumaticHub phub;
  // Compressor compressor;

  public Wrist() {
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
    // solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.wristSolenoidID);
    // phub = new PneumaticHub(Constants.pHubID);
    // compressor = new Compressor(Constants.compressorID, PneumaticsModuleType.REVPH);
    Timer.delay(1);
    //resetToAbsolutePosition();
  }

  public void resetToAbsolutePosition() {
    motor.setSelectedSensorPosition(wristEncoder.getAbsolutePosition());
  }

  public void setSpeed(double speed){
    // solenoid.set(true);
    motor.set(ControlMode.PercentOutput, speed);
  }
  
  public void setSpeed0ArbitraryFeedForward(){
    motor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0.05);
  }

  public double getClosedLoopError() {
    return pid.getPositionError();
  }

  public void up() {
    motor.set(ControlMode.PercentOutput, 0.5);
  }

  public void down() {
    motor.set(ControlMode.PercentOutput, -0.5);
  }

  public boolean isAtCurrentLimit() {
    return motor.getStatorCurrent() >= 25;
  }

  public double getAbsEncoderPos() {
    return wristEncoder.getAbsolutePosition();
  }

  public double getEncoderPos() {
    return wristEncoder.getPosition();
  }

  public double getPosition() {
    return motor.getSelectedSensorPosition();
  }

  public boolean globalWristMaxAngleUp() {
    return getEncoderPos() <= WristConstants.globalWristMaxAngleUp;
  }

  public boolean globalWristMaxAngleDown() {
    return getEncoderPos() >= WristConstants.globalWristMaxAngleDown; // Second part is to make it work if it loops back around to 0
  }

  public void setState(double state) {
    motor.set(ControlMode.PercentOutput, pid.calculate(getEncoderPos(), state));
    //motor.set(ControlMode.Position, state);
  }

  public boolean isAtSetpoint() {
    return pid.atSetpoint();
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
    if (wristEncoder.getPosition() <= 5 && wristEncoder.getPosition() >= 0) {
      wristEncoder.setPosition(360);
    }
    //SmartDashboard.putNumber("wrist current", motor.getStatorCurrent());
    if (isAtCurrentLimit()) {
      motor.set(ControlMode.PercentOutput, 0);
    }
    SmartDashboard.putNumber("wrist absolute", getAbsEncoderPos());
    SmartDashboard.putNumber("wrist pos", getPosition());
    SmartDashboard.putNumber("wrist encoder", getEncoderPos());
  }
}
