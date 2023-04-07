// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist; 

public class SetElevator extends CommandBase {
  Elevator elevator;
  int state;
  Timer timer;
  int pos;
  Wrist w;
  /** Creates a new SetElevator. */
  public SetElevator(Elevator e, int state_g, Wrist w) {
    elevator = e;
    state = state_g;
    this.w = w;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(e);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state == 0) {
      pos = Constants.ElevatorConstants.bottomPos;
    } else if (state == 1) {
      pos = Constants.ElevatorConstants.midPos;
    } else if (state == 2) {
      pos = Constants.ElevatorConstants.highPos;
    } else if (state == 3) {
      pos = Constants.ElevatorConstants.shelfPos;
    }
    
    if (elevator.getBottomLimits() && state == 0) {
      elevator.elevate(0);
      return;
    } else if (elevator.topLimitSwitch()) { // TODO: wtf!
      elevator.elevate(0);
      return;
    }

    if(state == 0){
      elevator.setState(0);
    }
    else if(state == 1 && w.getEncoderPos() >= WristConstants.wristConditionForElevatorMovement) {
      elevator.setState(1);
    }
    else if(state == 2 && w.getEncoderPos() >= WristConstants.wristConditionForElevatorMovement) {
      elevator.setState(2);
    }
    else if(state == 3){
      elevator.setState(3);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SmartDashboard.putBoolean("setElevator ended", true);
    if (elevator.getBottomLimits()){
      elevator.resetEncoderPos();
      elevator.elevate(0);
    // } else if (elevator.topLimitSwitch()) {
    //   elevator.resetEncoderPosTop();
    //   elevator.elevate(0);
    } else {
      elevator.elevate(0);
    }
    //elevator.setHoldPos();
    //elevator.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(elevator.getState() == state){
    //   return true;
    // }
    // return false;
    //return elevator.isAtSetpoint();
    if (elevator.getEncoderPos() >= pos-150 && elevator.getEncoderPos() <= pos+150) {
      //SmartDashboard.putBoolean("setElevator finished", true);
      return true;
    }
    return false;
  }
}
