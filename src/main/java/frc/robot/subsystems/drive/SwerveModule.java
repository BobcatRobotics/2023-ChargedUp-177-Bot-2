package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleInputs;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule extends SubsystemBase {
    private SwerveModuleIO io;
    public int moduleNumber;
    private SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

    public SwerveModule(SwerveModuleIO io){
        this.io = io;
        this.moduleNumber = io.moduleNumber;
    }
    
    public void setBrakeMode(boolean brakeMode){
        io.setBrakeMode(brakeMode);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        io.setSpeed(desiredState, isOpenLoop);
    }
    

    public void enableBrakeMode(boolean enable) {
        io.enableBrakeMode(enable);
    }

    private void setAngle(SwerveModuleState desiredState){
        io.setAngle(desiredState);
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(inputs.angleDegrees);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(inputs.canCoderMeasurment);
    }

    private void waitForCanCoder(){
        /*
         * Wait for up to 1000 ms for a good CANcoder signal.
         *
         * This prevents a race condition during program startup
         * where we try to synchronize the Falcon encoder to the
         * CANcoder before we have received any position signal
         * from the CANcoder.
         */
        io.waitForCanCoder();
    }
    // https://www.chiefdelphi.com/t/5013-the-trobots-2023-charged-up-open-alliance-build-thread/419112/37

    public void resetToAbsolute(){
        io.resetToAbsolute();
    }

    private void configAngleEncoder(){        
        io.configAngleEncoder();
    }

    private void configAngleMotor(){
        io.configAngleMotor();
    }

    private void configDriveMotor(){        
        io.configDriveMotor();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(inputs.driveSensorVelocity, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(inputs.driveSensorPosition, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
    public double getDistanceMeters(){
        return Conversions.falconToMeters(inputs.driveSensorPosition, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("SwerveModule", inputs);
    }
}