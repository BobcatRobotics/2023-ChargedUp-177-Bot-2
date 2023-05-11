package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModuleIOTalonFX extends SwerveModuleIO {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;
    private Rotation2d newAngle;
    private Rotation2d optimizedNewAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    public double CANcoderInitTime = 0.0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModuleIOTalonFX(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, Constants.Swerve.CANivore);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.Swerve.CANivore);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.Swerve.CANivore);
        configDriveMotor();

        lastAngle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    @Override
    public void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    @Override
    public void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        //Timer.delay(1);
        resetToAbsolute();
    }

    @Override
    public void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void resetToAbsolute(){
        waitForCanCoder();

        double absolutePosition = Conversions.degreesToFalcon(makePositiveDegrees(angleEncoder.getAbsolutePosition() - angleOffset.getDegrees()), Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
        //angleEncoder.setPosition(absolutePosition); // TODO: test potential fix
    }

    public double makePositiveDegrees(double anAngle) {
        double degrees = anAngle;
        degrees = degrees % 360;
        if (degrees < 0.0){
            degrees = degrees + 360;
        }
        return degrees;

    }

    public double makePositiveDegrees(Rotation2d anAngle){
        return makePositiveDegrees(anAngle.getDegrees());
    }

    @Override
    public void waitForCanCoder(){
        /*
         * Wait for up to 1000 ms for a good CANcoder signal.
         *
         * This prevents a race condition during program startup
         * where we try to synchronize the Falcon encoder to the
         * CANcoder before we have received any position signal
         * from the CANcoder.
         */
        for (int i = 0; i < 100; ++i) {
            angleEncoder.getAbsolutePosition();
            if (angleEncoder.getLastError() == ErrorCode.OK) {
                break;
            }
            Timer.delay(0.010);            
            CANcoderInitTime += 10;
        }
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }
    
    @Override
    public void enableBrakeMode(boolean enable) {
        if (enable) {
            mDriveMotor.setNeutralMode(NeutralMode.Brake);
        } else {
            mDriveMotor.setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d oldAngle = getAngle();
        angle = optimizeTurn(oldAngle, angle);  
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d optimizeTurn(Rotation2d oldAngle, Rotation2d newAngle){
        double steerAngle = makePositiveDegrees(newAngle);
        steerAngle %= (360);
        if (steerAngle < 0.0) {
            steerAngle += 360;
        }

        double difference = steerAngle - oldAngle.getDegrees();
        // Change the target angle so the difference is in the range [-360, 360) instead of [0, 360)
        if (difference >= 360) {
            steerAngle -= 360;
        } else if (difference < -360) {
            steerAngle += 360;
        }
        difference = steerAngle - oldAngle.getDegrees(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference >90 || difference < -90) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            steerAngle += 180;
        }

        this.newAngle = newAngle;
        this.optimizedNewAngle = Rotation2d.fromDegrees(makePositiveDegrees(steerAngle));

        return Rotation2d.fromDegrees(makePositiveDegrees(steerAngle));
    }

    @Override
    public void setBrakeMode(boolean brakeMode){
        mAngleMotor.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
        mDriveMotor.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public double getDistanceMeters(){
        return Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
    }

    public double getVelocityMeters() {
        return Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }

    public double getAngleVelocity() {
        return Conversions.falconToRPM(mAngleMotor.getSelectedSensorVelocity(), Constants.Swerve.angleGearRatio);
    }

    @Override
    public void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.drivePosition = getDistanceMeters();
        inputs.driveVelocity = getVelocityMeters();
        inputs.driveVelocityError = mDriveMotor.getClosedLoopError();
        inputs.driveCurrent = mDriveMotor.getStatorCurrent();
        inputs.driveTemperature = mDriveMotor.getTemperature();
        inputs.driveVoltage = mDriveMotor.getMotorOutputVoltage();
        inputs.drivePercentOutput = mDriveMotor.getMotorOutputPercent();
        inputs.angleDegrees = getAngle().getDegrees();
        inputs.angleRPM = getAngleVelocity();
        inputs.anglePositionError = mAngleMotor.getClosedLoopError();
        inputs.angleCurrent = mAngleMotor.getStatorCurrent();
        inputs.angleTemperature = mAngleMotor.getTemperature();
        inputs.angleVoltage = mAngleMotor.getMotorOutputVoltage();
        inputs.anglePercentOutput = mAngleMotor.getMotorOutputPercent();
        inputs.angleAbsolutePosition = Conversions.degreesToFalcon(makePositiveDegrees(angleEncoder.getAbsolutePosition() - angleOffset.getDegrees()), Constants.Swerve.angleGearRatio);
        if (lastAngle != null) {
            inputs.lastAngle = lastAngle.getDegrees();
        }
        if (newAngle != null) {
            inputs.newAngle = newAngle.getDegrees();
        }
        if (optimizedNewAngle != null) {
            inputs.optimizedNewAngle = optimizedNewAngle.getDegrees();
        }
        inputs.canCoderMeasurment = angleEncoder.getAbsolutePosition();
        inputs.driveSensorVelocity = mDriveMotor.getSelectedSensorVelocity();
        inputs.driveSensorPosition = mDriveMotor.getSelectedSensorPosition();
    }
}
