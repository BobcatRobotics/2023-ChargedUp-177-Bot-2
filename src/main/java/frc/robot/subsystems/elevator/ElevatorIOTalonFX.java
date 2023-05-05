package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX extends ElevatorIO {
    private WPI_TalonFX elevatorMotor;
    private DigitalInput topLimit;
    private DigitalInput bottomLimit;

    public ElevatorIOTalonFX() {
        elevatorMotor = new WPI_TalonFX(ElevatorConstants.elevatorMotorPort);
        topLimit = new DigitalInput(ElevatorConstants.topLimitPort);
        bottomLimit = new DigitalInput(ElevatorConstants.bottomLimitPort);

        elevatorMotor.configFactoryDefault();
        elevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        elevatorMotor.setSensorPhase(true);
        elevatorMotor.configNominalOutputForward(0, 20);
        elevatorMotor.configNominalOutputReverse(0, 20);
        elevatorMotor.configPeakOutputForward(1, 20);
        elevatorMotor.configPeakOutputReverse(-1, 20);//TODO: needs to be changes for comp
        elevatorMotor.configAllowableClosedloopError(0, 0, 20);
        //elevatorMotor.config_kF(0, 0, 20);
        elevatorMotor.config_kP(0, 0.25, 20);
        elevatorMotor.config_kI(0, 0, 20);
        elevatorMotor.config_kD(0, 0, 20);
        elevatorMotor.setInverted(false); // used to be true but we flipped motor direction 2/25/23
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        elevatorMotor.configNeutralDeadband(0.001, 20);
        elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
        elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
        elevatorMotor.configAllowableClosedloopError(0, 400, 20);
        // motion magic trapezoid configuration
        //elevatorMotor.configAllowableClosedloopError()
        elevatorMotor.configMotionCruiseVelocity(100000, 20); //needs to be tuned to robot
        elevatorMotor.configMotionAcceleration(75000, 20);
        //elevatorMotor.configMotionSCurveStrength(-20);
    }

    @Override
    public void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.elevatorPosition = elevatorMotor.getSelectedSensorPosition();
        inputs.elevatorVelocity = elevatorMotor.getSelectedSensorVelocity();
        inputs.bottomLimitSwitchHit = !bottomLimit.get();
        inputs.topLimitSwitchHit = !topLimit.get();
        inputs.elevatorCurrent = elevatorMotor.getStatorCurrent();
        inputs.elevatorTemperature = elevatorMotor.getTemperature();
        inputs.elevatorPercentOutput = elevatorMotor.getMotorOutputPercent();
        inputs.elevatorVoltage = elevatorMotor.getMotorOutputVoltage();
        inputs.elevatorPositionError = elevatorMotor.getClosedLoopError();
    }

    @Override
    public void setElevatorPosition(double position) {
        elevatorMotor.set(ControlMode.MotionMagic, position);
    }

    @Override
    public void setElevatorPercentOutput(double percent) {
        elevatorMotor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void resetToPosition(double position) {
        elevatorMotor.setSelectedSensorPosition(position);
    }

    @Override
    public void setElevatorPercentOutputWithArbitraryFeedforward(double percent, double feedforward) {
        elevatorMotor.set(ControlMode.PercentOutput, percent, DemandType.ArbitraryFeedForward, feedforward);
    }

}
