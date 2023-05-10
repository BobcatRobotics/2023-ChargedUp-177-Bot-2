package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModuleIO {
    public int moduleNumber;
    
    @AutoLog
    public static class SwerveModuleInputs {
        double drivePosition = 0.0;
        double driveVelocity = 0.0;
        double driveVelocityError = 0.0;
        double driveCurrent = 0.0;
        double driveTemperature = 0.0;
        double driveVoltage = 0.0;
        double drivePercentOutput = 0.0;
        double angleDegrees = 0.0;
        double angleRPM = 0.0;
        double anglePositionError = 0.0;
        double angleCurrent = 0.0;
        double angleTemperature = 0.0;
        double angleVoltage = 0.0;
        double anglePercentOutput = 0.0;
        double angleAbsolutePosition = 0.0;
        double lastAngle = 0.0;
        double newAngle = 0.0;
        double optimizedNewAngle = 0.0;
        double canCoderMeasurment = 0.0;
        double driveSensorVelocity = 0.0;
        double driveSensorPosition = 0.0;
    } 

    public void updateInputs(SwerveModuleInputsAutoLogged inputs) {}

    public void setBrakeMode(boolean brakeMode) {}

    // public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {}

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {}

    public void enableBrakeMode(boolean enable) {}

    public void setAngle(SwerveModuleState desiredState) {}

    public void waitForCanCoder() {}

    public void resetToAbsolute() {}

    public void configAngleEncoder() {}

    public void configAngleMotor() {}

    public void configDriveMotor() {}
}
