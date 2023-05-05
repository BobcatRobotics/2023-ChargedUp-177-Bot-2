package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmIO.ArmInputs;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();
    
    public Arm(ArmIO io) {
        this.io = io;
    }

    public void setSpeed(double speed) {
        io.setArmPercentOutput(speed);
    }

    public void setSpeed0ArbitraryFeedForward() {
        io.setArmPercentOutputArbitraryFeedForward(0, -0.02);
    }

    public void runArmOutSlow() {
        io.setArmPercentOutput(0.2);
    }
    public void stop() {
        io.setArmPercentOutput(0);
    }

    // public void holdPosition() {
    //     armMotor.set(ControlMode.Position, holdPosValue);
    // }

    // public void setHoldPos() {
    //     holdPosValue = armMotor.getSelectedSensorPosition();
    // }

    public void setState(double state) {
        // if (state == 0) {
        //     armMotor.set(ControlMode.MotionMagic, ArmConstants.pos0);
        //     SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        // } else if (state == 1) {
        //     armMotor.set(ControlMode.MotionMagic, ArmConstants.pos1);
        //     SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        // } else  if (state == 2) {
        //     armMotor.set(ControlMode.MotionMagic, ArmConstants.pos2);
        //     SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        // } else if (state == 3) {
        //     armMotor.set(ControlMode.MotionMagic, ArmConstants.bottomPickup);
        //     SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        // }else if (state == 4){
        //     armMotor.set(ControlMode.MotionMagic, ArmConstants.minNonCollidingExtention+400);
        // }
    }
    public void setPos(int pos) {
        io.setArmPosition(pos);
    }

    public boolean isAtStowedLimit() {
        //return !armLimit.get();
        return isAtMaxTuck();
    }

    // public int getState() {
    //     double pos = armMotor.getSelectedSensorPosition();
    //     if (pos <= 256) pos = 0;
    //     return (int) Math.ceil(pos/4096);
    // }

    // public boolean isAtTopLimit() {
    //     return armMotor.getSelectedSensorPosition() >= Constants.ArmConstants.topLimit;
    // }

    // public boolean isAtBottomLimit() {
    //     return armMotor.getSelectedSensorPosition() <= Constants.ArmConstants.bottomLimit;
    // }

    // public boolean isAtConstrictedBottomLimit() {
    //     return armMotor.getSelectedSensorPosition() <= Constants.ArmConstants.constrictedBottomLimit;
    // }

    // public boolean isAtSetpoint() {
    //     return armMotor.isMotionProfileFinished();
    // }
    
    public boolean isAtSetpoint() {
        return inputs.armIsAtSetpoint;
        //return armMotor.isMotionProfileFinished();
    }

    public boolean isAtMaxTuck() {
        return getEncoderPos() <= ArmConstants.startingArmPos || getEncoderPos() >= 355; // Second part is to make it work if it loops back around to 360
    }

    public boolean isAtMaxExtension() {
        return getEncoderPos() >= ArmConstants.groundPickupArm;
    }

    // public double absoluteEncoderVal() {
    //     return absoluteEncoder.getIntegratedSensorAbsolutePosition();
    // }
    
    public boolean isAtCurrentLimit() {
        return inputs.armCurrent >= 50.0;
    }


    public double getAbsEncoderPos() {
        return inputs.armAbsolutePosition;
    }

    public double getEncoderPos() {
        return inputs.armPosition;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // This method will be called once per scheduler run
        if (inputs.armPosition >= ArmConstants.trueArmMaxExtension) {
            io.resetToPosition(360 - inputs.armPosition);
        }
        //armMotor.setSelectedSensorPosition(getEncoderPos());
        SmartDashboard.putNumber("arm absolute", getAbsEncoderPos());
        SmartDashboard.putNumber("arm encoder", getEncoderPos());
        SmartDashboard.putNumber("arm pos", inputs.armPosition);
    }
}
