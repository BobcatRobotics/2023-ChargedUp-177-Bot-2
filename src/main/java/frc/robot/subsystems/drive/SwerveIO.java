package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveIO {
    @AutoLog
    public static class SwerveInputs {
        double yaw = 0.0;
        double pitch = 0.0;
        double roll = 0.0;
        double commandedXSpeed = 0.0;
        double commandedYSpeed = 0.0;
        double commandedRotationSpeed = 0.0;
        SwerveModuleState[] modStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
        SwerveModulePosition[] modPositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    }

    public void updateInputs(SwerveInputsAutoLogged inputs) {}

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {}
    
    public void drive(ChassisSpeeds targetSpeeds) {}

    public void enableBrakeMode(boolean enable) {}

    public void setModuleStates(SwerveModuleState[] desiredStates) {}

    public void resetModulesToAbsolute() {}

    public void zeroGyro() {}

    public void reverseZeroGyro() {}

    public void configToX() {}

    public void setBrakeMode(boolean brake) {}

}
