package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public abstract class ArmIO {
    @AutoLog
    public static class ArmInputs {
        double armPosition = 0.0;
        double armAbsolutePosition = 0.0;
        double armVelocity = 0.0;
        double armCurrent = 0.0;
        double armTemp = 0.0;
        double armVoltage = 0.0;
        double armPercentOutput = 0.0;
        boolean armIsAtSetpoint = false;
    }

    public void updateInputs(ArmInputsAutoLogged inputs) {}

    public void setArmPosition(double position) {}

    public void setArmPercentOutput(double percent) {}

    public void resetToPosition(double position) {}

    public void setArmPercentOutputArbitraryFeedForward(double percent, double feedforward) {}
}
