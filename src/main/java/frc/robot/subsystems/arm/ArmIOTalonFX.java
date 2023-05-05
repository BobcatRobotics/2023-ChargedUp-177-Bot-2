package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmIOTalonFX extends ArmIO {
    private WPI_TalonFX armMotor;
    private CANCoder armEncoder;
    private CANCoderConfiguration encoderConfig;
    private PIDController pid;

    public ArmIOTalonFX() {
        armMotor = new WPI_TalonFX(Constants.ArmConstants.armMotorPort);
        armEncoder = new CANCoder(Constants.armCanCoderID);
        encoderConfig = new CANCoderConfiguration();
        //armLimit = new DigitalInput(Constants.ArmConstants.stowedLimitSwitch);
        //absoluteEncoder = new TalonFXSensorCollection(armMotor);
        armEncoder.configFactoryDefault();
        armEncoder.configAllSettings(encoderConfig);
        armEncoder.configMagnetOffset(-ArmConstants.armOffset);
        //armEncoder.configFeedbackCoefficient(4096/360, "falcons", SensorTimeBase.Per100Ms_Legacy);
        armEncoder.setPositionToAbsolute();

        armMotor.configFactoryDefault();
        armMotor.configRemoteFeedbackFilter(armEncoder.getDeviceID(), RemoteSensorSource.CANCoder, 0, 20);
        armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 20);
        armMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        armMotor.setSensorPhase(true);
        armMotor.configNominalOutputForward(0, 20);
        armMotor.configNominalOutputReverse(0, 20);
        armMotor.configPeakOutputForward(0.5, 20);
        armMotor.configPeakOutputReverse(-0.5, 20);
        armMotor.configAllowableClosedloopError(0, 0, 20);
        armMotor.config_kF(0, 0, 20);
        armMotor.config_kP(0, 0.1, 20); // 0.275
        armMotor.config_kI(0, 0, 20);
        armMotor.config_kD(0, 0, 20);
        armMotor.setInverted(true);
        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.configNeutralDeadband(0.001, 20);
        armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, 20);
        armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, 20);
        armMotor.configMotionCruiseVelocity(30000, 20); //needs to be tuned to robot
        armMotor.configMotionAcceleration(24000, 20);
        armMotor.configAllowableClosedloopError(0, 200, 20);

        pid = new PIDController(0.0375, 0, 0);
        pid.setTolerance(4);
        // holdPosValue = armMotor.getSelectedSensorPosition();
        Timer.delay(1);
        //resetToAbsolutePosition();
    }

    @Override
    public void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.armPosition = armEncoder.getPosition();
        inputs.armAbsolutePosition = armEncoder.getAbsolutePosition();
        inputs.armCurrent = armMotor.getStatorCurrent();
        inputs.armPercentOutput = armMotor.getMotorOutputPercent();
        inputs.armTemp = armMotor.getTemperature();
        inputs.armVelocity = armEncoder.getVelocity();
        inputs.armVoltage = armMotor.getMotorOutputVoltage();
        inputs.armIsAtSetpoint = pid.atSetpoint();
        inputs.armPositionError = pid.getPositionError();
    }

    @Override
    public void setArmPosition(double position) {
        armMotor.set(ControlMode.PercentOutput, pid.calculate(armEncoder.getPosition(), position));
    }

    @Override
    public void setArmPercentOutput(double percent) {
        armMotor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void resetToPosition(double position) {
        //armMotor.setSelectedSensorPosition(position);
        armEncoder.setPosition(position);
    }

    @Override
    public void setArmPercentOutputArbitraryFeedForward(double percent, double feedforward) {
        armMotor.set(ControlMode.PercentOutput, percent, DemandType.ArbitraryFeedForward, feedforward);
    }
}
