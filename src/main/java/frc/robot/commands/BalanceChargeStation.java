// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalancingConstants;
import frc.robot.subsystems.Swerve;



public class BalanceChargeStation extends CommandBase {
  private boolean isContinuous = true; //if false, the command will end after the robot is balanced
  

  private PIDController pid; // see Ramsete.java for info on PIDControllers
  private Swerve dt;
  double calc;

  
  

  /** Creates a new BalanceChargeStation. */
  public BalanceChargeStation(Swerve dt) {
    pid = new PIDController(BalancingConstants.kP, BalancingConstants.kI, BalancingConstants.kD);
    pid.setTolerance(BalancingConstants.kToleranceDegrees);
    this.dt = dt;
    pid.setSetpoint(BalancingConstants.kSetpoint);
  }


  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("docking debugging", "started balancing");
    SmartDashboard.putBoolean("BalanceChargeStation", false);

    //start pid loop
    //devide to reduce the sensitivity
    //if pitch within 5(?) degrees of 0, stop pid loop
    calc = -pid.calculate(dt.getPitch());
    //for every degree of pitch, drive forward 1/15 of a meter
    dt.drive(new Translation2d(calc/BalancingConstants.sensitivity, 0), 0, true, false);
    SmartDashboard.putString("docking debugging", "balancing");
    SmartDashboard.putNumber("error", calc);

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("docking debugging", "balanced");
    SmartDashboard.putBoolean("BalanceChargeStation", true);
    dt.setBrakeMode(true);
    dt.drive(new Translation2d(0, 0), 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    
    if(!isContinuous){//if we are not in continuous mode    
      return pid.atSetpoint();//then end the command when the robot is balanced
    }else{ 
      return false; //otherwise the command will run indefinetly
    }
  }
}