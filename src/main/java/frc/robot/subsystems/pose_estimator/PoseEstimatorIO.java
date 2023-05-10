package frc.robot.subsystems.pose_estimator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.vision.Limelight;

public abstract class PoseEstimatorIO {
    @AutoLog
    public static class PoseEstimatorInputs {
        // Pose2d currentPose = new Pose2d();
        // Pose2d visionMeasurementCameraFront = new Pose2d();
        // Pose2d visionMeasurementCameraBack = new Pose2d();
        double latencyCameraFront = 0.0;
        double latencyCameraBack = 0.0;
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        // Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0, 0, 0);
        // SwerveModulePosition[] modulePositions = {};
        // Rotation2d rotation = new Rotation2d();
    }

    public void updateInputs(PoseEstimatorInputsAutoLogged inputs) {}

    public void addVisionMeasurement(Pose2d pose, double timeStampSeconds, Vector<N3> visionStdDevs, boolean isBackCamera) {}

    public void setCurrentPose(Rotation2d rot, SwerveModulePosition[] modPositions, Pose2d pose) {}

    public void update(Rotation2d rot, SwerveModulePosition[] modPositions) {}
}
