package frc.robot.subsystems.pose_estimator;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Limelight;

public class PoseEstimatorIOLimelight extends PoseEstimatorIO {
private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(0.9));

  private final SwerveDrivePoseEstimator poseEstimator;
  //private PhotonVision photon = new PhotonVision();

  private Pose2d visionMeasurementCameraFront = new Pose2d();
  private double latencyCameraFront = 0.0;
  private Pose2d visionMeasurementCameraBack = new Pose2d();
  private double latencyCameraBack = 0.0;


  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
  


  //private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;
  private boolean sawTag = false;

  private final double FIELD_LENGTH_METERS = 16.54175;
  private final double FIELD_WIDTH_METERS = 8.0137;

  public PoseEstimatorIOLimelight(Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modulePositionSupplier) {
    this.rotationSupplier = rotationSupplier;
    this.modulePositionSupplier = modulePositionSupplier;

    poseEstimator = new SwerveDrivePoseEstimator(
      Constants.Swerve.swerveKinematics, 
      this.rotationSupplier.get(),
      this.modulePositionSupplier.get(), 
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs
    );
  }

  @Override
  public void updateInputs(PoseEstimatorInputsAutoLogged inputs) {
    inputs.currentPose = poseEstimator.getEstimatedPosition();
    inputs.visionMeasurementCameraFront = visionMeasurementCameraFront;
    inputs.visionMeasurementCameraBack = visionMeasurementCameraBack;
    inputs.latencyCameraFront = latencyCameraFront;
    inputs.latencyCameraBack = latencyCameraBack;
    inputs.visionMeasurementStdDevs = visionMeasurementStdDevs;
    inputs.modulePositions = modulePositionSupplier.get();
    inputs.rotation = rotationSupplier.get();
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timeStampSeconds, Vector<N3> visionStdDevs, boolean isBackCamera) {
    this.visionMeasurementStdDevs = visionStdDevs;
    if (!isBackCamera) {
        latencyCameraFront = Timer.getFPGATimestamp() - timeStampSeconds;
        visionMeasurementCameraFront = pose;
    } else {
        latencyCameraBack = Timer.getFPGATimestamp() - timeStampSeconds;
        visionMeasurementCameraBack = pose;
    }
    poseEstimator.addVisionMeasurement(pose, timeStampSeconds, visionStdDevs);
  }

  @Override
  public void setCurrentPose(Pose2d pose) {
    poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), pose);
  }

  @Override
  public void update() {
    poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());
  }

}
