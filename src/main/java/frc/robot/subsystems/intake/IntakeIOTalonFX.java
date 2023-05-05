package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

public class IntakeIOTalonFX extends IntakeIO {
    private WPI_TalonFX motor; 

    public IntakeIOTalonFX() {
        motor = new WPI_TalonFX(Constants.intakeMotorID);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(true);
    }

    @Override
    public void updateInputs(IntakeInputsAutoLogged inputs) {
        double intakeCurrent = motor.getStatorCurrent();
        double intakePercentOutput = motor.getMotorOutputPercent();
        double intakeVoltage = motor.getMotorOutputVoltage();
        double intakeTemperature = motor.getTemperature();
    }

    @Override
    public void setIntakePercentOutput(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }
}
