package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Climber;

public class ClimbUp extends SequentialCommandGroup {
    public ClimbUp(Climber climber) {
        addRequirements(climber);
        addCommands(new InstantCommand(climber::setBalanceLevel));
        addCommands(new WaitCommand(1).andThen(climber::setFloorLevel));
    }
}