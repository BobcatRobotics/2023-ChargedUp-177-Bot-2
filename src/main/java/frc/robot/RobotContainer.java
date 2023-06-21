package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.PoseEstimation;
import frc.robot.commands.ArmControls;
import frc.robot.commands.DriveBack;
import frc.robot.commands.DriveBackInverse;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.ElevatorControls;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeOutFullSpeed;
import frc.robot.commands.SmallDrive;
import frc.robot.commands.SpinInPlace;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WaitUntilTime;
import frc.robot.commands.AutoAlign.MidLeft;
import frc.robot.commands.AutoAlign.MidMid;
import frc.robot.commands.AutoAlign.MidRight;
import frc.robot.commands.AutoAlign.TopLeft;
import frc.robot.commands.AutoAlign.TopMid;
import frc.robot.commands.AutoAlign.TopRight;
import frc.robot.commands.Autos.BalanceChargeStation;
import frc.robot.commands.Autos.MountAndBalance;
import frc.robot.commands.Autos.MountAndBalanceInverse;
import frc.robot.commands.Autos.MountAndBalanceInverseScore;
import frc.robot.commands.LEDs.Blink;
import frc.robot.commands.Presets.IntakeInConstantly;
import frc.robot.commands.Presets.RunIntake;
import frc.robot.commands.Presets.RunWrist;
import frc.robot.commands.Presets.SetArm;
import frc.robot.commands.Presets.SetElevator;
import frc.robot.commands.Presets.StartingConfig;
import frc.robot.commands.Presets.intakeStop;
import frc.robot.commands.Presets.Procedures.GrabFromHPChute;
import frc.robot.commands.Presets.Procedures.ScoreHigh;
import frc.robot.commands.Presets.Procedures.ScoreMid;
import frc.robot.commands.Presets.Procedures.TopSuck;
import frc.robot.commands.Presets.Procedures.autoCarry;
import frc.robot.commands.Presets.Procedures.falcon5;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BlinkinLEDs;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
//import frc.robot.autos.RedHighCone6PickupBalance;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // for the xbox controller buttons
    // private Constants.ButtonHashtable bh = new Constants.ButtonHashtable();

    /* Controllers */
    private final Joystick driver = new Joystick(2);
    private final Joystick rotate = new Joystick(0);
    private final Joystick strafe = new Joystick(1);
    private final Joystick buttonBoards = new Joystick(3);

    // /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */

    private final JoystickButton xButton = new JoystickButton(driver, 5);
    // private final JoystickButton alignRobot = new JoystickButton(driver, 2);
    // private final JoystickButton zeroGyro = new JoystickButton(driver, 4);

    // ruffy buttons
    private final JoystickButton ruffy0 = new JoystickButton(rotate, 1);
    private final JoystickButton ruffy1 = new JoystickButton(strafe, 1);
    // driver button board
    // TODO: check if these are the right sides
    private final JoystickButton driverLeft = new JoystickButton(buttonBoards, 2);
    private final JoystickButton driverRight = new JoystickButton(buttonBoards, 1);

    // private final JoystickButton alignRobot = new JoystickButton(driver, 1);

    // operator buttons
    private final JoystickButton OpTopLeft = new JoystickButton(buttonBoards, 3);
    private final JoystickButton OpTopMid = new JoystickButton(buttonBoards, 4);
    private final JoystickButton OpTopRight = new JoystickButton(buttonBoards, 5);
    private final JoystickButton OpBottomLeft = new JoystickButton(buttonBoards, 6);
    private final JoystickButton OpBottomMid = new JoystickButton(buttonBoards, 7);
    private final JoystickButton OpBottomRight = new JoystickButton(buttonBoards, 8);
    // private final JoystickButton Op = new JoystickButton(buttonBoards, 9);
    // private final JoystickButton Op = new JoystickButton(buttonBoards, 10);
    // private final JoystickButton Op = new JoystickButton(buttonBoards, 11);

    private final JoystickButton leftBumper = new JoystickButton(driver, 5); // left bumper
    private final JoystickButton rightBumper = new JoystickButton(driver, 6);// right bumper
    private final JoystickButton a = new JoystickButton(driver, 2);
    private final JoystickButton b = new JoystickButton(driver, 3);
    private final JoystickButton x = new JoystickButton(driver, 1); // purple
    private final JoystickButton y = new JoystickButton(driver, 4); // yellow
    private final JoystickButton righttrigger = new JoystickButton(driver, 8);
    private final JoystickButton lefttrigger = new JoystickButton(driver, 7);
    private final JoystickButton back = new JoystickButton(driver, 9);
    private final JoystickButton start = new JoystickButton(driver, 10);
    private final POVButton DUp = new POVButton(driver, 0);
    private final POVButton DLeft = new POVButton(driver, 270);
    private final POVButton DDown = new POVButton(driver, 180);
    private final POVButton DRight = new POVButton(driver, 90);
    /* Subsystems */

    public static Limelight m_LimelightFront = new Limelight("limelight-front");
    public static Limelight m_LimelightBack = new Limelight("limelight-back");
    public static Swerve s_Swerve = new Swerve();
    public static PoseEstimator swervePoseEstimator = new PoseEstimator(s_Swerve::getYaw, s_Swerve::getModulePositions,
            m_LimelightFront, m_LimelightBack);

    private final Elevator m_Elevator = new Elevator();
    private final Intake m_Intake = new Intake();
    private final Arm m_Arm = new Arm();
    private final Wrist m_Wrist = new Wrist();
    private final BlinkinLEDs m_LEDs = new BlinkinLEDs();
    // private final PWMLEDs m_LEDs = new PWMLEDs();

    /* Commands */
    private final Command elevatorControls = new ElevatorControls(m_Elevator, driver, m_Arm);
    private final Command armControls = new ArmControls(m_Arm, driver, m_Elevator);

    // private final SequentialCommandGroup chargestation = new
    // MountAndBalance(s_Swerve);

    // private final Command align = new AlignToTarget(s_Swerve,
    // m_Limelight).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).repeatedly();
    // private final Command align = new AlignToTarget(s_Swerve, m_Limelight);
    // private final RedHighCone6PickupBalance redHighCone6PickupBalance = new
    // RedHighCone6PickupBalance(s_Swerve, m_Limelight);
    // private PathPlannerTest pathPlannerTest;
    private static SwerveAutoBuilder swerveAutoBuilder;

    /* SendableChooser */
    private static SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    public static Command NoMove1HighCone = null;
    public static Command OneCenterBalance = null;
    public static Command RedClean2Path = null;
    public static Command BlueClean2Path = null;
    public static Command BlueTwoPieceDirty = null;
    public static Command RedTwoPieceDirty = null;
    public static Command CenterTwoBalance = null;
    public static Command ThreePieceDirty = null;
    public static Command TwoPieceCenterBalance = null;

    public void setUpAutos() {
        // Sendable Chooser Setup
        // autoChooser.setDefaultOption("Red High Cone 6 Pickup & Balance",
        // redHighCone6PickupBalance);
        setUpEventMap();
        // pathPlannerTest = new PathPlannerTest();
        // autoChooser.setDefaultOption("1HighDirtyBalance",
        // buildAuto(PathPlanner.loadPathGroup("Score1HighCubeRightBalance", new
        // PathConstraints(4.5, 3))));
        // autoChooser.addOption("1HybridCleanBalance",
        // buildAuto(PathPlanner.loadPathGroup("Score1LeftBalance", new
        // PathConstraints(4, 3))));
        // autoChooser.addOption("CleanBalance",
        // buildAuto(PathPlanner.loadPathGroup("LeftBalance", new PathConstraints(4,
        // 3))));
        // autoChooser.addOption("DirtyBalance",
        // buildAuto(PathPlanner.loadPathGroup("RightBalance", new PathConstraints(4,
        // 3))));
        // autoChooser.addOption("CenterBalance",
        // PathPlanner.loadPathGroup("CenterBalance", new PathConstraints(4, 3)));
        // autoChooser.addOption("Score1CenterBalance",
        // PathPlanner.loadPathGroup("Score1CenterBalance", new PathConstraints(4, 3)));
        // autoChooser.addOption("1HybridDirtyBalance",
        // buildAuto(PathPlanner.loadPathGroup("Score1RightBalance", new
        // PathConstraints(4, 3))));
        // autoChooser.addOption("1HighCleanBalance",
        // buildAuto(PathPlanner.loadPathGroup("Score1HighCubeLeftBalance", new
        // PathConstraints(4.5, 3))));
        // autoChooser.addOption("Score1HighCubeCenterBalance",
        // PathPlanner.loadPathGroup("Score1HighCubeCenterBalance", new
        // PathConstraints(4.5, 3)));
        // autoChooser.addOption("Score1HighCubeCleanNoBalance",
        // PathPlanner.loadPathGroup("ScoreHighCubeCleanNoBalance", new
        // PathConstraints(4.5, 3)));
        // autoChooser.addOption("1HighDirtyNoBalance",
        // buildAuto(PathPlanner.loadPathGroup("ScoreHighCubeDirtyNoBalance", new
        // PathConstraints(4.5, 3))));
        // autoChooser.addOption("NoMove1High",
        // buildAuto(PathPlanner.loadPathGroup("NoMoveScore1High", new
        // PathConstraints(0, 0))));
        autoChooser.setDefaultOption("NoMove1HighCone", NoMove1HighCone);
        // autoChooser.addOption("NoTurn1HighCenterBalance",
        // buildAuto(PathPlanner.loadPathGroup("NoTurnScore1HighCenterBalance", new
        // PathConstraints(4, 3))));
        autoChooser.addOption("Clean1.5Balance",
                buildAuto(PathPlanner.loadPathGroup("Score1HighCubePickupLeftBalance", new PathConstraints(4.5, 3))));
        autoChooser.addOption("BlueDirty1.5Balance",
                buildAuto(PathPlanner.loadPathGroup("Blue1.5DirtyBalance", new PathConstraints(4.5, 3))));
        autoChooser.addOption("RedDirty1.5Balance",
                buildAuto(PathPlanner.loadPathGroup("Red1.5DirtyBalance", new PathConstraints(4.5, 3))));
        // autoChooser.addOption("Clean1.5",
        // buildAuto(PathPlanner.loadPathGroup("Score1HighCubePickupLeftNoBalance", new
        // PathConstraints(4.5, 3))));
        // autoChooser.addOption("Dirty1.5",
        // buildAuto(PathPlanner.loadPathGroup("Score1HighCubePickupRightNoBalance", new
        // PathConstraints(4.5, 3))));
        // autoChooser.addOption("Clean2Balance",
        // buildAuto(PathPlanner.loadPathGroup("2PieceBalanceClean", new
        // PathConstraints(4.5, 3))));
        autoChooser.addOption("RedClean2", RedClean2Path);
        autoChooser.addOption("BlueClean2", BlueClean2Path);
        autoChooser.addOption("BlueDirty2", BlueTwoPieceDirty); // TODO: rename with no balance
        autoChooser.addOption("RedDirty2", RedTwoPieceDirty);
        // autoChooser.addOption("Clean2Balance",
        // buildAuto(PathPlanner.loadPathGroup("2PieceHighBalanceClean", new
        // PathConstraints(4.5, 3))));
        // autoChooser.addOption("Clean3Hybrid",
        // buildAuto(PathPlanner.loadPathGroup("3PieceHybridClean", new
        // PathConstraints(4.5, 3))));
        // autoChooser.addOption("PPTestBalance",
        // buildAuto(PathPlanner.loadPathGroup("PPTestBalance", new PathConstraints(2,
        // 2))));
        // autoChooser.addOption("Center1.5Balance",
        // buildAuto(PathPlanner.loadPathGroup("1.5CenterBalance", new
        // PathConstraints(3.5, 3.0))));
        autoChooser.addOption("Center1Balance", OneCenterBalance);
        autoChooser.addOption("Center2Balance", CenterTwoBalance);
        autoChooser.addOption("Dirty3", ThreePieceDirty);
        autoChooser.addOption("Center2Balance", TwoPieceCenterBalance);
        // autoChooser.addOption("2CleanHighConeBalance",
        // buildAuto(PathPlanner.loadPathGroup("2CleanHighConeBalance", new
        // PathConstraints(3.0, 3.0))));
        // autoChooser.addOption("PathPlanner Test w/ Events", new
        // SequentialCommandGroup(Swerve.followTrajectoryCommand(PathPlanner.loadPath("New
        // Path", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        // Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)), true)));
        // autoChooser.addOption("charge station", chargestation);
        SmartDashboard.putData(autoChooser);
    }

    public void setUpEventMap() {
        Constants.AutoConstants.eventMap.clear();
        Constants.AutoConstants.eventMap.put("chargeStation", new MountAndBalance(s_Swerve));
        // Constants.AutoConstants.eventMap.put("align", new
        // AlignToTargetAutos(s_Swerve, m_Limelight));
        Constants.AutoConstants.eventMap.put("highPreset", new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist));
        Constants.AutoConstants.eventMap.put("intakeGround", new ParallelCommandGroup(
                new TopSuck(m_Elevator, m_Arm, m_Intake, m_Wrist),
                new IntakeInConstantly(m_Intake)));

        // Potential fix if starting config fails to properly stow
        // Constants.AutoConstants.eventMap.put("startingConfig", new
        // ParallelDeadlineGroup(new StartingConfig(m_Elevator, m_Arm, m_Wrist), new
        // SequentialCommandGroup(new WaitCommand(2.5), new StartingConfig(m_Elevator,
        // m_Arm, m_Wrist))));
        Constants.AutoConstants.eventMap.put("conePoop", new IntakeInConstantly(m_Intake));
        Constants.AutoConstants.eventMap.put("stopIntake", new intakeStop(m_Intake));

        Constants.AutoConstants.eventMap.put("startingConfig", new StartingConfig(m_Elevator, m_Arm, m_Wrist));
        Constants.AutoConstants.eventMap.put("autoStowe", new autoCarry(m_Wrist, m_Arm));
        // Constants.AutoConstants.eventMap.put("flickWrist", new
        // InstantCommand(m_Wrist::wristSolenoidON));
        Constants.AutoConstants.eventMap.put("intakeOut", new IntakeOut(m_Intake));// new ParallelRaceGroup(new
                                                                                   // IntakeOut(), new WaitCommand(5)));
        Constants.AutoConstants.eventMap.put("intakeOutFullSpeed", new IntakeOutFullSpeed(m_Intake));
        Constants.AutoConstants.eventMap.put("driveBack", new DriveBack(s_Swerve)); // TODO: Actually is driving
                                                                                    // forward, my bad
        Constants.AutoConstants.eventMap.put("spinInPlace", new SpinInPlace(s_Swerve));
        Constants.AutoConstants.eventMap.put("waitHalfSec", new WaitCommand(0.5));
        Constants.AutoConstants.eventMap.put("smallDrive", new SmallDrive(s_Swerve));
        Constants.AutoConstants.eventMap.put("scoreCubeHigh", new SequentialCommandGroup(
                // new InstantCommand(m_Wrist::wristSolenoidON),
                new ParallelRaceGroup(new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist), new WaitCommand(2)),
                // new InstantCommand(m_Wrist::wristSolenoidON),
                new WaitCommand(0.2),
                new IntakeOutFullSpeed(m_Intake),
                new StartingConfig(m_Elevator, m_Arm, m_Wrist)));
        Constants.AutoConstants.eventMap.put("falcon5", new falcon5(m_Arm, m_Wrist));
        Constants.AutoConstants.eventMap.put("scoreConeHigh", new SequentialCommandGroup(
                // new InstantCommand(m_Wrist::wristSolenoidON),
                new ParallelRaceGroup(new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist), new WaitCommand(2)),
                new WaitCommand(0.25),
                new IntakeOut(m_Intake),
                new StartingConfig(m_Elevator, m_Arm, m_Wrist)));
        Constants.AutoConstants.eventMap.put("setElevatorMid",
                new ParallelRaceGroup(new ScoreMid(m_Elevator, m_Arm, m_Wrist), new WaitCommand(1.5)));

        Constants.AutoConstants.eventMap.put("scoreCubeMid", new SequentialCommandGroup(
                // new InstantCommand(m_Wrist::wristSolenoidON),
                new ParallelRaceGroup(new ScoreMid(m_Elevator, m_Arm, m_Wrist), new WaitCommand(1.5)),
                // new InstantCommand(m_Wrist::wristSolenoidON),
                new WaitCommand(0.2),
                new IntakeOut(m_Intake, .1),
                new StartingConfig(m_Elevator, m_Arm, m_Wrist)));
        Constants.AutoConstants.eventMap.put("driveBackInverse", new DriveBackInverse(s_Swerve));
        Constants.AutoConstants.eventMap.put("chargeStationInverse", new MountAndBalanceInverse(s_Swerve));
        Constants.AutoConstants.eventMap.put("chargeStationInverseScore",
                new MountAndBalanceInverseScore(s_Swerve, m_Elevator, m_Arm, m_Wrist, m_Intake));
        Constants.AutoConstants.eventMap.put("balanceChargeStationInverse",
                new BalanceChargeStation(s_Swerve, true, true));
        // First "clean" = grid
        // Second "clean" = column
        Constants.AutoConstants.eventMap.put("highCleanClean", new SequentialCommandGroup(
                new DriveToPoseCommand(s_Swerve, () -> PoseEstimation.grid3[2],
                        () -> swervePoseEstimator.getCurrentPose(), true),
                new ParallelRaceGroup(new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist), new WaitCommand(2)),
                new IntakeOutFullSpeed(m_Intake),
                new StartingConfig(m_Elevator, m_Arm, m_Wrist)));
        Constants.AutoConstants.eventMap.put("highCleanMid", new ParallelDeadlineGroup(
                new WaitUntilTime(14.5, new IntakeOutFullSpeed(m_Intake)), new SequentialCommandGroup(
                        new DriveToPoseCommand(s_Swerve, () -> PoseEstimation.grid3[1],
                                () -> swervePoseEstimator.getCurrentPose(), true),
                        new ParallelRaceGroup(new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist), new WaitCommand(2)),
                        new IntakeOutFullSpeed(m_Intake),
                        new StartingConfig(m_Elevator, m_Arm, m_Wrist))));
        Constants.AutoConstants.eventMap.put("highDirtyMid", new ParallelDeadlineGroup(
                new WaitUntilTime(14.5, new IntakeOutFullSpeed(m_Intake)), new SequentialCommandGroup(
                        new DriveToPoseCommand(s_Swerve, () -> PoseEstimation.grid1[1],
                                () -> swervePoseEstimator.getCurrentPose(), true),
                        new ParallelRaceGroup(new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist), new WaitCommand(2)),
                        new IntakeOutFullSpeed(m_Intake),
                        new StartingConfig(m_Elevator, m_Arm, m_Wrist))));
        Constants.AutoConstants.eventMap.put("finalScoreHigh",
                new ParallelDeadlineGroup(new WaitUntilTime(14, new IntakeOutFullSpeed(m_Intake)),
                        new SequentialCommandGroup(new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist),
                                new IntakeOutFullSpeed(m_Intake))));
        Constants.AutoConstants.eventMap.put("finalOuttake", new WaitUntilTime(14.5, new IntakeOutFullSpeed(m_Intake)));
        Constants.AutoConstants.eventMap.put("birdyScore", new SequentialCommandGroup(
                new GrabFromHPChute(m_Elevator, m_Arm, m_Wrist),
                new ParallelCommandGroup(new WaitUntilTime(14.5, new IntakeOutFullSpeed(m_Intake)),
                        new IntakeOutFullSpeed(m_Intake))));
        Constants.AutoConstants.eventMap.put("birdyScore2",
                new ParallelCommandGroup(new MountAndBalanceInverse(s_Swerve), new SequentialCommandGroup(
                        new GrabFromHPChute(m_Elevator, m_Arm, m_Wrist),
                        new WaitUntilTime(14.5, new IntakeOutFullSpeed(m_Intake)))));
    }

    // public void printHashMap() {
    // SmartDashboard.putString("eventMap",
    // Constants.AutoConstants.eventMap.toString());
    // }

    public void zeroGyro() {
        s_Swerve.zeroGyro();
    }

    public void reverseZeroGyro() {
        s_Swerve.reverseZeroGyro();
    }

    // returns the angle of the joystick in degrees
    public double getJoystickAngle() {
        return ((Math.atan2(rotate.getRawAxis(Joystick.AxisType.kY.value),
                rotate.getRawAxis(Joystick.AxisType.kX.value)) * 180 / Math.PI) + 360) % 360;
    }

    // public void displayGyro(){
    // SmartDashboard.putNumber("pitch", s_Swerve.getPitch());
    // SmartDashboard.putNumber("yaw", s_Swerve.getRoll());
    // }

    public Command getDefaultCommand() {
        return s_Swerve.getDefaultCommand();
    }

    public void resetToAbsolute() {
        s_Swerve.resetModulesToAbsolute();
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        /*
         * Default commands should be scheduled here if they should run all the time
         * (auto and teleop).
         * Make sure that none of the default commands require the same subsystems that
         * you intend to
         * use during autonomous, because they will interrupt the autonomous command. If
         * you want a default
         * command to run just during teleop, schedule it in the scheduleDefaultTeleop
         * method and cancel
         * it in the cancelDefaultTeleop method.
         */
        // Configure the button bindings
        // CameraServer.startAutomaticCapture(0);
        setUpEventMap();
        NoMove1HighCone = buildAuto(PathPlanner.loadPathGroup("NoMoveScore1HighCone", new PathConstraints(0, 0)));
        OneCenterBalance = buildAuto(PathPlanner.loadPathGroup("1CenterBalance", new PathConstraints(2, 2)));
        RedClean2Path = buildAuto(PathPlanner.loadPathGroup("RedClean2", new PathConstraints(2.0, 2.0)));
        BlueClean2Path = buildAuto(PathPlanner.loadPathGroup("BlueClean2", new PathConstraints(2.0, 2.0)));
        BlueTwoPieceDirty = buildAuto(PathPlanner.loadPathGroup("Blue2PieceDirty", new PathConstraints(2.5, 2.5)));
        RedTwoPieceDirty = buildAuto(PathPlanner.loadPathGroup("Red2PieceDirty", new PathConstraints(2.0, 2.0)));
        CenterTwoBalance = buildAuto(PathPlanner.loadPathGroup("2CenterBalance", new PathConstraints(2.0, 2.0)));
        ThreePieceDirty = buildAuto(PathPlanner.loadPathGroup("BlueDirt3", new PathConstraints(2.5, 2.5)));
        TwoPieceCenterBalance = buildAuto(PathPlanner.loadPathGroup("1.5CenterBalance", new PathConstraints(2.5, 2.5)));
        m_LimelightFront.setAlliance(DriverStation.getAlliance());
        m_LimelightBack.setAlliance(DriverStation.getAlliance());
        configureButtonBindings();
    }

    public void scheduleDefaultTeleop() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -strafe.getRawAxis(Joystick.AxisType.kY.value)
                                * Math.abs(strafe.getRawAxis(Joystick.AxisType.kY.value)),
                        () -> -strafe.getRawAxis(Joystick.AxisType.kX.value)
                                * Math.abs(strafe.getRawAxis(Joystick.AxisType.kX.value)),
                        () -> -rotate.getRawAxis(Joystick.AxisType.kX.value),
                        () -> false, // () -> robotCentric.getAsBoolean() //always field centric,
                        () -> -rotate.getRawAxis(Joystick.AxisType.kZ.value) * 0.2,
                        () -> -strafe.getRawAxis(Joystick.AxisType.kZ.value) * 0.2 // fine tune strafing supplier
                ));

        m_Arm.setDefaultCommand(armControls);
        m_Elevator.setDefaultCommand(elevatorControls);
        m_Intake.setDefaultCommand(new RunIntake(m_Intake, driver, m_LEDs));
        m_Wrist.setDefaultCommand(new RunWrist(m_Wrist, righttrigger, rightBumper));

        // Configure the button bindings
        configureButtonBindings();
    }

    public void cancelDefaultTeleop() {
        s_Swerve.getDefaultCommand().cancel();
        m_Arm.getDefaultCommand().cancel();
        m_Elevator.getDefaultCommand().cancel();
        m_Intake.getDefaultCommand().cancel();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        // resetToAbsolute.onTrue(new InstantCommand(() ->
        // s_Swerve.resetModulesToAbsolute()));
        // alignRobot.whileTrue(align);
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        ruffy0.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        ruffy1.onTrue(new InstantCommand(() -> s_Swerve.configToX()));

        driverLeft.onTrue(new Blink(m_LEDs, false));
        driverRight.onTrue(new Blink(m_LEDs, true));
        // y.onTrue(new Blink(m_LEDs, false));
        // x.onTrue(new Blink(m_LEDs, true));
        // driverLeft.onTrue(new BlinkPWM(m_LEDs, false));
        // driverRight.onTrue(new BlinkPWM(m_LEDs, true));

        OpTopLeft.onTrue(new TopLeft(s_Swerve, swervePoseEstimator, m_Elevator, m_Arm, m_Wrist, m_Intake)
                .until(this::baseDriverControlsMoved));
        OpTopMid.onTrue(new TopMid(s_Swerve, swervePoseEstimator, m_Elevator, m_Arm, m_Wrist, m_Intake)
                .until(this::baseDriverControlsMoved));
        OpTopRight.onTrue(new TopRight(s_Swerve, swervePoseEstimator, m_Elevator, m_Arm, m_Wrist, m_Intake)
                .until(this::baseDriverControlsMoved));
        OpBottomLeft.onTrue(new MidLeft(s_Swerve, swervePoseEstimator, m_Elevator, m_Arm, m_Wrist, m_Intake)
                .until(this::baseDriverControlsMoved));
        OpBottomMid.onTrue(new MidMid(s_Swerve, swervePoseEstimator, m_Elevator, m_Arm, m_Wrist, m_Intake)
                .until(this::baseDriverControlsMoved));
        OpBottomRight.onTrue(new MidRight(s_Swerve, swervePoseEstimator, m_Elevator, m_Arm, m_Wrist, m_Intake)
                .until(this::baseDriverControlsMoved));

        // lefttrigger.whileTrue(new InstantCommand(m_Intake::runIntakeOut));
        // leftBumper.whileTrue(new InstantCommand(m_Intake::runIntakeIn));

        // righttrigger.whileTrue(new InstantCommand(m_Wrist::down));
        // rightBumper.whileTrue(new InstantCommand(m_Wrist::up));

        // if(driver.getPOV() == 0){
        // new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist);
        // }
        DUp.onTrue(new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist).until(this::operatorMoved));
        // if(driver.getPOV() == 180){
        // new StartingConfig(m_Elevator, m_Arm);
        // }
        DDown.onTrue(new StartingConfig(m_Elevator, m_Arm, m_Wrist).until(this::operatorMoved));
        // if(driver.getPOV() == 270){
        // new ScoreMid(m_Elevator, m_Arm, m_Intake, m_Wrist);
        // }
        DLeft.onTrue(new ScoreMid(m_Elevator, m_Arm, m_Wrist).until(this::operatorMoved));

        b.onTrue(new GrabFromHPChute(m_Elevator, m_Arm, m_Wrist).until(this::operatorMoved)); // chute

        // a.onTrue(new ForwardSuck(m_Elevator, m_Arm,
        // m_Wrist).until(this::operatorMoved));
        a.onTrue(new TopSuck(m_Elevator, m_Arm, m_Intake, m_Wrist).until(this::operatorMoved)); // TopSuck? This command
                                                                                                // should be named
                                                                                                // BottomSuck
        // y.onTrue(new GrabFromHPShelf(m_Elevator, m_Arm,
        // m_Wrist).until(this::operatorMoved));
        // x.onTrue(new VerticalCone(m_Elevator, m_Arm,
        // m_Wrist).until(this::operatorMoved));

        // x.onTrue(new InstantCommand(m_LEDs::setPurple));
        // y.onTrue(new InstantCommand(m_LEDs::setYellow));

        // back.whileTrue(new InstantCommand(m_Intake::runIntakeOutFull));

        // alignRobot.whileTrue(align);
        start.whileTrue(driveToPose().until(this::baseDriverControlsMoved));
    }

    public Command driveToPose() {
        return new DriveToPoseCommand(s_Swerve, this::closestGrid, swervePoseEstimator::getCurrentPose, true);
    }

    public Pose2d getSelectedNode() {
        return Constants.PoseEstimation.scoringPositions.get(1);
    }

    public Pose2d closestGrid() {
        List<Pose2d> poses = List.of(
                Constants.PoseEstimation.grid1[1],
                Constants.PoseEstimation.grid2[1],
                Constants.PoseEstimation.grid3[1]);

        switch (poses.indexOf(swervePoseEstimator.getCurrentPose().nearest(poses)) + 1) {
            case 1:
                return Constants.PoseEstimation.grid1[1];
            case 2:
                return Constants.PoseEstimation.grid2[1];
            case 3:
                return Constants.PoseEstimation.grid3[1];
            default:
                return Constants.PoseEstimation.grid2[1];
        }
    }

    public boolean anythingPressed() {
        return operatorMoved() || baseDriverControlsMoved();
        // return operatorMoved();
    }

    public boolean operatorMoved() {
        return Math.abs(driver.getRawAxis(1)) >= 0.1 || Math.abs(driver.getRawAxis(3)) >= 0.1;
    }

    public boolean baseDriverControlsMoved() {
        return Math.abs(rotate.getRawAxis(Joystick.AxisType.kX.value)) >= 0.1
                || Math.abs(strafe.getRawAxis(Joystick.AxisType.kX.value)) >= 0.1
                || Math.abs(strafe.getRawAxis(Joystick.AxisType.kY.value)) >= 0.1;
    }

    public void turnOffLeds() {
        m_LEDs.turnOff();
    }

    public void setLedsBlack() {
        m_LEDs.setBlack();
    }

    public static Command buildAuto(List<PathPlannerTrajectory> trajs) {
        swerveAutoBuilder = new SwerveAutoBuilder(
                swervePoseEstimator::getCurrentPose,
                swervePoseEstimator::setCurrentPose,
                Constants.Swerve.swerveKinematics,
                new PIDConstants(Constants.AutoConstants.kPXController, 0, 0),
                new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
                s_Swerve::setModuleStates,
                Constants.AutoConstants.eventMap,
                true,
                s_Swerve);

        return swerveAutoBuilder.fullAuto(trajs);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // s_Swerve.resetOdometry(new Pose2d(0, 0, s_Swerve.getYaw()));
        // return new MountAndBalance(s_Swerve); //autoChooser.getSelected();
        return autoChooser.getSelected();
    }

    public static Command getAutoChooserResult() {
        return autoChooser.getSelected();
    }
}
