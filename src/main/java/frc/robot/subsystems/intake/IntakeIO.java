package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public abstract class IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        double intakeCurrent = 0.0;
        double intakePercentOutput = 0.0;
        double intakeVoltage = 0.0;
        double intakeTemperature = 0.0;
    }

    public void updateInputs(IntakeInputsAutoLogged inputs) {}

    public void setIntakePercentOutput(double percent) {}
}
