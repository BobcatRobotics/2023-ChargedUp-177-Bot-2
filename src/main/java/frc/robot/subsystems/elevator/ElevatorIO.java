package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public abstract class ElevatorIO {
    @AutoLog
    public static class ElevatorInputs {
        double elevatorPosition = 0.0;
        double elevatorVelocity = 0.0;
        double elevatorPositionError = 0.0;
        double elevatorCurrent = 0.0;
        double elevatorTemperature = 0.0;
        double elevatorVoltage = 0.0;
        double elevatorPercentOutput = 0.0;
        boolean bottomLimitSwitchHit = false;
        boolean topLimitSwitchHit = false;
    }

    public void updateInputs(ElevatorInputsAutoLogged inputs) {}

    public void setElevatorPosition(double position) {}

    public void setElevatorPercentOutput(double percent) {}

    public void resetToPosition(double position) {}

    public void setElevatorPercentOutputWithArbitraryFeedforward(double percent, double feedforward) {}
}
