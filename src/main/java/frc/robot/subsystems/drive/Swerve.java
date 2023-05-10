package frc.robot.subsystems.drive;

import frc.robot.Util.GeometryUtils;
import frc.robot.subsystems.drive.SwerveIO.SwerveInputs;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class Swerve extends SubsystemBase {
    //public SwerveDriveOdometry swerveOdometry;
    //public PoseEstimator poseEstimator;
    private SwerveIO io;
    private SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();


    public Swerve(SwerveIO io) {
        this.io = io;
    }

    public double getPitch(){
        return inputs.pitch;
    }
    public double getRoll(){
        return inputs.roll;
    }

    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        io.drive(translation, rotation, fieldRelative, isOpenLoop);
    }    
        
    


    public void drive(ChassisSpeeds targetSpeeds) {
        io.drive(targetSpeeds);
        
    }

    public void enableBrakeMode(boolean enable) {
        io.enableBrakeMode(enable);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        io.setModuleStates(desiredStates);
    }    

    // public Pose2d getPose() {
    //     //return swerveOdometry.getPoseMeters();
    //     return poseEstimator.getCurrentPose();
    // }

    // public void resetOdometry(Pose2d pose) {
    //     //swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    //     poseEstimator.setCurrentPose(pose);
    // }

    public SwerveModuleState[] getModuleStates(){
        // SwerveModuleState[] states = new SwerveModuleState[4];
        // for(SwerveModule mod : mSwerveMods){
        //     states[mod.moduleNumber] = mod.getState();
        // }
        // return states;
        return inputs.modStates;
    }

    public SwerveModulePosition[] getModulePositions(){
        // SwerveModulePosition[] positions = new SwerveModulePosition[4];
        // for(SwerveModule mod : mSwerveMods){
        //     positions[mod.moduleNumber] = mod.getPosition();
        // }
        // return positions;
        return inputs.modPositions;
    }

    public void resetModulesToAbsolute() {
        io.resetModulesToAbsolute();
    }

    public void zeroGyro(){
        io.zeroGyro();
    }

    public void reverseZeroGyro() {
        io.reverseZeroGyro();
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(inputs.yaw);
    }


    //sets the wheels in an x shape
    public void configToX(){
        io.configToX();
    }

    

    // public void resetOdometryAutos() {
    //     path1.initialize();
    // }

    /** @param break true for break, false for coast */
    public void setBrakeMode(boolean brake){
        io.setBrakeMode(brake);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Swerve", inputs);
        Logger.getInstance().recordOutput("Swerve/Swerve Module States", getModuleStates());
        // double[] botpose = limelight.botPose(); 
        // if (limelight.tv() == 1.0) {
        //     resetOdometry(new Pose2d(botpose[0], botpose[1], getYaw()));
        // }
        // swerveOdometry.update(getYaw(), getModulePositions());
        //poseEstimator.periodic();

        if (DriverStation.isDisabled()) {
            resetModulesToAbsolute();
        }

        
        
        // SmartDashboard.putNumber("swerveOdometry Pose X Meters", poseEstimator.getCurrentPose().getX());
        // SmartDashboard.putNumber("swerveOdometry Pose Y Meters", poseEstimator.getCurrentPose().getY());
        // SmartDashboard.putNumber("swerveOdometry Pose Rotation Degrees", poseEstimator.getCurrentPose().getRotation().getDegrees());
    }
}
