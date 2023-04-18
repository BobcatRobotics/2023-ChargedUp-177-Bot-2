// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve;

public class DriveToPoseCommand extends CommandBase {
  /** Creates a new DriveToPoseCommand. */
  private static final double TRANSLATION_TOLERANCE = 0.1;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
    Constants.AutoConstants.kMaxSpeedMetersPerSecond
  );
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
    Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
    Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared
  );

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final Swerve swerve;
  private final Supplier<Pose2d> poseProvider;
  private final Supplier<Pose2d> goalPoseSupplier;
  private final boolean useAllianceColor;

  public DriveToPoseCommand(Swerve swerve, Supplier<Pose2d> goalPoseSupplier, Supplier<Pose2d> poseProvider, boolean useAllianceColor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.goalPoseSupplier = goalPoseSupplier;
    this.poseProvider = poseProvider;
    this.useAllianceColor = useAllianceColor;

    xController = new ProfiledPIDController(Constants.PoseEstimation.kPXController, 0, 0, DEFAULT_XY_CONSTRAINTS);
    yController = new ProfiledPIDController(Constants.PoseEstimation.kPYController, 0, 0, DEFAULT_XY_CONSTRAINTS);
    thetaController = new ProfiledPIDController(Constants.PoseEstimation.kPThetaController, 0, 0, DEFAULT_OMEGA_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController.setTolerance(THETA_TOLERANCE);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.drive(new Translation2d(), 0, true, false);
    resetPIDControllers();
    Pose2d pose = goalPoseSupplier.get();
    if (useAllianceColor && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      Translation2d transformedTranslation = new Translation2d(pose.getX(), FieldConstants.width - pose.getY());
      Rotation2d transformedHeading = pose.getRotation().times(-1);
      pose = new Pose2d(transformedTranslation, transformedHeading);
    }
    xController.setGoal(pose.getX());
    yController.setGoal(pose.getY());
    thetaController.setGoal(pose.getRotation().getRadians());
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  private void resetPIDControllers() {
    Pose2d robotPose = poseProvider.get();
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    thetaController.reset(robotPose.getRotation().getRadians());
  }

  private void setGoals(Pose2d goal) {
    if (useAllianceColor && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      Translation2d transformedTranslation = new Translation2d(goal.getX(), FieldConstants.width - goal.getY());
      Rotation2d transformedHeading = goal.getRotation().times(-1);
      goal = new Pose2d(transformedTranslation, transformedHeading);
    }
    xController.setGoal(goal.getX());
    yController.setGoal(goal.getY());
    thetaController.setGoal(goal.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = poseProvider.get();
    ArrayList<Pose2d> chargeStation = new ArrayList<>();

    for (Pose2d edge : Constants.PoseEstimation.chargeStation) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        chargeStation.add(new Pose2d(edge.getX(), Constants.FieldConstants.width - edge.getY(), edge.getRotation().times(-1)));
      } else {
        chargeStation.add(new Pose2d(edge.getX(), edge.getY(), edge.getRotation()));
      }
    }
    
    if (robotPose.getX() >= chargeStation.get(2).getX()) {
      yController.setGoal(poseProvider.get().getY());
      // if (robotPose.getY() <= chargeStation.get(1).getY()) {
      //   yController.setGoal(chargeStation.get(1).getY());
      // } else if (robotPose.getY() >= chargeStation.get(0).getY()) {
      //   yController.setGoal(chargeStation.get(0).getY());
      // }
    } else {
      this.setGoals(goalPoseSupplier.get());
    }

    double xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    double ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    double thetaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      thetaSpeed = 0;
    }

    swerve.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, robotPose.getRotation())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0.0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atGoal();
  }
}