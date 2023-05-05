// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pose_estimator.PoseEstimator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.Presets.Procedures.ScoreHigh;
import frc.robot.commands.Presets.Procedures.ScoreMid;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class MidLeft extends SequentialCommandGroup {
  /** Creates a new TopMid. */
  Pose2d desiredpos;
  PoseEstimator sPose;
  Pose2d desiredPose;

  public MidLeft(Swerve s, PoseEstimator sPose, Elevator e, Arm a, Wrist w, Intake i) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.sPose = sPose;
   
  
  
  

  
    
    addCommands(
      new DriveToPoseCommand(s, this::getGrid, sPose::getCurrentPose, true),
      new ScoreMid(e, a, w)
    );
  }
  
  public Pose2d getGrid(){
    List<Pose2d> poses = List.of(
      Constants.PoseEstimation.grid1[1],
      Constants.PoseEstimation.grid2[1],
      Constants.PoseEstimation.grid3[1]
    );

    switch (poses.indexOf(sPose.getCurrentPose().nearest(poses))+1){
      case 1:
          return Constants.PoseEstimation.grid1[2];
      case 2:
          return Constants.PoseEstimation.grid2[2];
      case 3:
          return Constants.PoseEstimation.grid3[2];
      default:
          return Constants.PoseEstimation.grid2[2];
      }
  }
}

