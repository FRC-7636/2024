package frc.robot.commands.GROUP_CMD;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BackToOrigin extends ParallelCommandGroup {
    public BackToOrigin(Climber climber, Intake intake, Shooter shooter){
        addCommands(new InstantCommand(climber::setFloorLevel, climber));
        addCommands(new InstantCommand(intake::backToZero, intake));
        addCommands(new InstantCommand(shooter::originAngle, shooter));
        addCommands(new InstantCommand(shooter::stopAll));
    }
}