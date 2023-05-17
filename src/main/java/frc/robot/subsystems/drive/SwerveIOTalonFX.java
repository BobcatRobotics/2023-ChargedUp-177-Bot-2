package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class SwerveIOTalonFX extends SwerveIO {
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public double rot;

    private double xSpeed;
    private double ySpeed;
    private double rotationSpeed;

    public SwerveIOTalonFX() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.CANivore);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(new SwerveModuleIOTalonFX(0, Constants.Swerve.Mod0.constants)),
            new SwerveModule(new SwerveModuleIOTalonFX(1, Constants.Swerve.Mod1.constants)),
            new SwerveModule(new SwerveModuleIOTalonFX(2, Constants.Swerve.Mod2.constants)),
            new SwerveModule(new SwerveModuleIOTalonFX(3, Constants.Swerve.Mod3.constants))
        };

        Timer.delay(1.0);
        //SmartDashboard.putBoolean("resetModsToAbsCalled", true);
        resetModulesToAbsolute();
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        xSpeed = translation.getX();
        ySpeed = translation.getY();
        rotationSpeed = rotation;
        
        ChassisSpeeds desiredSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            getYaw()
        )
        : new ChassisSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation
        );

        //desiredSpeeds = correctForDynamics(desiredSpeeds);

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        rot = rotation;
    }

    @Override
    public void drive(ChassisSpeeds targetSpeeds) {
        xSpeed = targetSpeeds.vxMetersPerSecond;
        ySpeed = targetSpeeds.vyMetersPerSecond;
        rotationSpeed = targetSpeeds.omegaRadiansPerSecond;

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
        
    }

    @Override
    public void enableBrakeMode(boolean enable) {
        for (SwerveModule mod : mSwerveMods) {
            mod.enableBrakeMode(enable);
        }
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    @Override
    public void resetModulesToAbsolute() {
        //zeroGyro();
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void zeroGyro(){
        gyro.setYaw(0);
    }

    @Override
    public void reverseZeroGyro() {
        gyro.setYaw(180);
    }

    @Override
    public void configToX(){
        mSwerveMods[0].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(45))), true);
        mSwerveMods[1].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(315))), true);
        mSwerveMods[2].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(315))), true);
        mSwerveMods[3].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(45))), true);
    }

    @Override
    /** @param break true for break, false for coast */
    public void setBrakeMode(boolean brake){
        for(SwerveModule mod : mSwerveMods){
            mod.setBrakeMode(brake);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    @Override
    public void updateInputs(SwerveInputsAutoLogged inputs) {
        inputs.yaw = getYaw().getDegrees();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
        inputs.commandedXSpeed = xSpeed;
        inputs.commandedYSpeed = ySpeed;
        inputs.commandedRotationSpeed = rotationSpeed;
        for(SwerveModule mod : mSwerveMods){
            inputs.velocityMPS[mod.moduleNumber] = mod.getState().speedMetersPerSecond;
            inputs.distanceMeters[mod.moduleNumber] = mod.getPosition().distanceMeters;
            inputs.angleDegrees[mod.moduleNumber] = mod.getState().angle.getDegrees();
        }
    }
}
