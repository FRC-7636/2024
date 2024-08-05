package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class AutoIntakeStop extends SequentialCommandGroup {
    public AutoIntakeStop(Shooter shooter, Intake intake) {
        addCommands(new InstantCommand(intake::stopAll));
        addCommands(new InstantCommand(intake::backToZero));
        addCommands(new InstantCommand(shooter::stopTransport));
        addCommands(new InstantCommand(shooter::standby));
    }
}