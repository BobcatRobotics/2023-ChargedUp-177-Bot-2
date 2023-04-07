// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
  /** Creates a new SetArm. */
  Arm arm;
  double state;
  Timer timer;
  double pos;

  // public SetArm(Arm a, double )

  public SetArm(Arm a, double state_g) {
    state = state_g;
    arm = a;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.isAtStowedLimit() && state == ArmConstants.startingArmPos) {
      arm.setSpeed(0);
    }
    
    arm.setPos((int) state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SmartDashboard.putBoolean("arm interrupted", interrupted);
    //arm.setClosedLoopSpeed(0);
    if (arm.getEncoderPos() > 50) {
      arm.setSpeed0ArbitraryFeedForward();
    } else {
      arm.setSpeed(0);
    }
    // if(arm.isAtBottomLimit()){
    //   arm.resetEncoder();
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (state == 0) {
    //   pos = Constants.ArmConstants.pos0;
    // } else if (state == 1) {
    //   pos = Constants.ArmConstants.pos1;
    // } else if (state == 2) {
    //   pos = Constants.ArmConstants.pos2;
    // } else if (state == 3) {
    //   pos = Constants.ArmConstants.bottomPickup;
    // }
    // if(arm.getState() == state){
    //   return true;
    // }
    // return false;
    // if (arm.getPos() >= pos-200 && arm.getPos() <= pos) {

    //   // SmartDashboard.putBoolean("elevator timer finished", timer.hasElapsed(5));

    //   return true;
    // }
    return arm.isAtSetpoint();
  }
}
