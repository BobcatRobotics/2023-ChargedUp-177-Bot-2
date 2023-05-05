package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void elevate(double speed) {
    if(speed == 0 && !getBottomLimits()) {
      //elevatorMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, -0.03);
      // elevatorMotor.set(ControlMode.PercentOutput, 0);
      io.setElevatorPercentOutputWithArbitraryFeedforward(speed, -0.03);
    } else if(speed == 0 && getBottomLimits()) {
      //elevatorMotor.set(ControlMode.PercentOutput, 0);
      io.setElevatorPercentOutput(0);
    } else{
      //elevatorMotor.set(ControlMode.PercentOutput, speed);
      io.setElevatorPercentOutput(speed);
    }
    
  }
  public double getSpeed(){
    //return elevatorMotor.getSelectedSensorVelocity();
    return inputs.elevatorVelocity;
  } 
  public boolean getStopped(){
    return getSpeed() == 0;
  }
  // public void holdPosition() {
  //   elevatorMotor.set(ControlMode.Position, holdPosValue);
  // }

  // public void holdPosition(double pos) {
  //   elevatorMotor.set(ControlMode.Position, pos);
  // }

  public double getPIDError(){
    return inputs.elevatorPositionError;
    //return elevatorMotor.getClosedLoopError();
  }

  // public void setHoldPos() {
  //   holdPosValue = elevatorMotor.getSelectedSensorPosition();
  // }

  public void resetEncoderPos() {
    //elevatorMotor.setSelectedSensorPosition(0);
    io.resetToPosition(0);
  }

  // public boolean isAtSetpoint() {
  //   return elevatorMotor.isMotionProfileFinished();
  // }

  // public boolean getTopLimits() {
  //   return !topLimit.get();
  // }

  public boolean getBottomLimits() {
    return inputs.bottomLimitSwitchHit;
  }
  public double getEncoderPos() {
    return inputs.elevatorPosition;
  }

  public boolean topLimitSwitch() {
    return inputs.topLimitSwitchHit;
  }

  public void setState(int state) {
    if (state == 0) {
      io.setElevatorPosition(ElevatorConstants.bottomPos);
      //elevatorMotor.set(ControlMode.MotionMagic, ElevatorConstants.bottomPos);
      // holdPosValue = ElevatorConstants.pos0;
      // holdPosition();
      // SmartDashboard.putString("elevator error", "State: " + state + ", Error: " + getPIDError());
    } else if (state == 1) {
      io.setElevatorPosition(ElevatorConstants.midPos);
      //elevatorMotor.set(ControlMode.MotionMagic, ElevatorConstants.midPos);
      // holdPosValue = ElevatorConstants.pos1;
      // holdPosition();
      // SmartDashboard.putString("elevator error", "State: " + state + ", Error: " + getPIDError());
    } else if (state == 2) {
      io.setElevatorPosition(ElevatorConstants.highPos);
      //elevatorMotor.set(ControlMode.MotionMagic, ElevatorConstants.highPos);
      // holdPosValue = ElevatorConstants.pos2;
      // holdPosition();
      // SmartDashboard.putString("elevator error", "State: " + state + ", Error: " + getPIDError());
    } else if (state == 3) {
      io.setElevatorPosition(ElevatorConstants.shelfPos);
      //elevatorMotor.set(ControlMode.MotionMagic, ElevatorConstants.shelfPos);
      // holdPosValue = ElevatorConstants.pos2;
      // holdPosition();
      // SmartDashboard.putString("elevator error", "State: " + state + ", Error: " + getPIDError());
    }
  }

  // public int getState() {
  //   double pos = elevatorMotor.getSelectedSensorPosition();
  //   if (pos <= 256) pos = 0;
  //   return (int) Math.ceil(pos/4096);
  // }

  // public double getEncoder() {
  //   return elevatorMotor.getSelectedSensorPosition();
  // }

  public boolean isAtCurrentLimit() {
    //return elevatorMotor.getStatorCurrent() >= 80.0;
    return false;
  }

  public void resetEncoderPosTop() {
    //elevatorMotor.setSelectedSensorPosition(-236710);
    io.resetToPosition(-236710);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);

    if (getBottomLimits()){
      resetEncoderPos();
    //   holdPosValue = 0;
    //   //elevatorMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0.06);
    //   elevatorMotor.set(ControlMode.PercentOutput, 0);
    }
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("bottom limits", getBottomLimits());
    SmartDashboard.putBoolean("top limits", topLimitSwitch());
    SmartDashboard.putNumber("elevator encoder", getEncoderPos());
    //SmartDashboard.putBoolean("top limits", topLimitSwitch());
  }
}