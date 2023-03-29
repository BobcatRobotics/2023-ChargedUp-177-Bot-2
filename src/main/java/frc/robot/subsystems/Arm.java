package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private WPI_TalonFX armMotor;
    private CANCoder armEncoder;
    private CANCoderConfiguration encoderConfig;
    //private TalonFXSensorCollection absoluteEncoder;
    //private DigitalInput armLimit;

    // private double holdPosValue;
    
    public Arm() {
        armMotor = new WPI_TalonFX(Constants.ArmConstants.armMotorPort);
        armEncoder = new CANCoder(Constants.armCanCoderID);
        encoderConfig = new CANCoderConfiguration();
        //armLimit = new DigitalInput(Constants.ArmConstants.stowedLimitSwitch);
        //absoluteEncoder = new TalonFXSensorCollection(armMotor);
        armEncoder.configFactoryDefault();
        armEncoder.configAllSettings(encoderConfig);
        armEncoder.configMagnetOffset(-ArmConstants.armOffset);
        armEncoder.setPositionToAbsolute();

        armMotor.configFactoryDefault();
        armMotor.configRemoteFeedbackFilter(armEncoder.getDeviceID(), RemoteSensorSource.CANCoder, 0, 20);
        armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 20);
        armMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        armMotor.setSensorPhase(true);
        armMotor.configNominalOutputForward(0, 20);
        armMotor.configNominalOutputReverse(0, 20);
        armMotor.configPeakOutputForward(0.25, 20);
        armMotor.configPeakOutputReverse(-0.25, 20);
        armMotor.configAllowableClosedloopError(0, 0, 20);
        armMotor.config_kF(0, 0, 20);
        armMotor.config_kP(0, 0.275, 20);
        armMotor.config_kI(0, 0, 20);
        armMotor.config_kD(0, 0, 20);
        armMotor.setInverted(true);
        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.configNeutralDeadband(0.001, 20);
        armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
        armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
        armMotor.configMotionCruiseVelocity(30000, 20); //needs to be tuned to robot
        armMotor.configMotionAcceleration(24000, 20);
        armMotor.configAllowableClosedloopError(0, 200, 20);

        // holdPosValue = armMotor.getSelectedSensorPosition();
        Timer.delay(1);
        //resetToAbsolutePosition();
    }

    public void resetToAbsolutePosition() {
        armMotor.setSelectedSensorPosition(armEncoder.getAbsolutePosition());
        
    }

    public void setSpeed(double speed) {
        armMotor.set(ControlMode.PercentOutput, speed);
    }
    public void runArmOutSlow() {
        armMotor.set(ControlMode.PercentOutput, 0.2);
    }
    public void stop() {
        armMotor.set(0);
    }

    // public void holdPosition() {
    //     armMotor.set(ControlMode.Position, holdPosValue);
    // }

    // public void setHoldPos() {
    //     holdPosValue = armMotor.getSelectedSensorPosition();
    // }

    public void setState(int state) {
        // if (state == 0) {
        //     armMotor.set(ControlMode.MotionMagic, ArmConstants.pos0);
        //     SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        // } else if (state == 1) {
        //     armMotor.set(ControlMode.MotionMagic, ArmConstants.pos1);
        //     SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        // } else  if (state == 2) {
        //     armMotor.set(ControlMode.MotionMagic, ArmConstants.pos2);
        //     SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        // } else if (state == 3) {
        //     armMotor.set(ControlMode.MotionMagic, ArmConstants.bottomPickup);
        //     SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        // }else if (state == 4){
        //     armMotor.set(ControlMode.MotionMagic, ArmConstants.minNonCollidingExtention+400);
        // }
    }
    public void setPos(int pos) {
        armMotor.set(ControlMode.MotionMagic, pos);
        SmartDashboard.putString("arm error", "State: " + pos + ", Error: " + getArmPIDError());
    }
    public double getArmPIDError(){
        return armMotor.getClosedLoopError();
    }
    public boolean isAtStowedLimit() {
        //return !armLimit.get();
        return false;
    }

    public int getState() {
        double pos = armMotor.getSelectedSensorPosition();
        if (pos <= 256) pos = 0;
        return (int) Math.ceil(pos/4096);
    }

    // public double getPos() {
    //     return armMotor.getSelectedSensorPosition();
    // }

    // public boolean isAtTopLimit() {
    //     return armMotor.getSelectedSensorPosition() >= Constants.ArmConstants.topLimit;
    // }

    // public boolean isAtBottomLimit() {
    //     return armMotor.getSelectedSensorPosition() <= Constants.ArmConstants.bottomLimit;
    // }

    // public boolean isAtConstrictedBottomLimit() {
    //     return armMotor.getSelectedSensorPosition() <= Constants.ArmConstants.constrictedBottomLimit;
    // }

    public boolean isAtSetpoint() {
        return armMotor.isMotionProfileFinished();
    }

    public boolean isAtMaxTuck() {
        return getEncoderPos() <= ArmConstants.startingArmPos;
    }

    public boolean isAtMaxExtension() {
        return getEncoderPos() >= ArmConstants.groundPickupArm;
    }

    // public double absoluteEncoderVal() {
    //     return absoluteEncoder.getIntegratedSensorAbsolutePosition();
    // }
    
    public boolean isAtCurrentLimit() {
        return armMotor.getStatorCurrent() >= 50.0;
    }

    public void resetEncoder() {
        armMotor.setSelectedSensorPosition(0);
    }

    public double getAbsEncoderPos() {
        return armEncoder.getAbsolutePosition();
    }

    public double getEncoderPos() {
        return armEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (armEncoder.getPosition() >= ArmConstants.trueArmMaxExtension) {
            armEncoder.setPosition(360 - armEncoder.getPosition());
        }
        if (isAtStowedLimit()) {
            resetEncoder();
        }
        SmartDashboard.putNumber("arm absolute", getAbsEncoderPos());
        SmartDashboard.putNumber("arm encoder", getEncoderPos());
        //SmartDashboard.putNumber("arm pos", getPos());
    }
}
