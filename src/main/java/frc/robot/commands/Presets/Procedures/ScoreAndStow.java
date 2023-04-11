package frc.robot.commands.Presets.Procedures;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeOutFullSpeed;
import frc.robot.commands.Presets.StartingConfig;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class ScoreAndStow extends SequentialCommandGroup {
    private Elevator elevator;
    private Arm arm;
    private Wrist wrist;
    private Intake intake;

    public ScoreAndStow(Elevator e, Arm a, Wrist w, Intake i) {
        elevator = e;
        arm = a;
        wrist = w;
        intake = i;
        addRequirements(e, a, w, i);

        addCommands(
            new IntakeOutFullSpeed(intake),
            new StartingConfig(elevator, arm, wrist)
        );
    }
}
