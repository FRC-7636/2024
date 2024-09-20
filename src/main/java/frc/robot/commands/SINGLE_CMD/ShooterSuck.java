package frc.robot.commands.SINGLE_CMD;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

public class ShooterSuck extends SequentialCommandGroup {
    public 
    ShooterSuck(Shooter shooter) {
        addCommands(new InstantCommand(() -> shooter.setPosition(ShooterConstants.Control.NEARSHOOT_POSITION)));
        addCommands(new InstantCommand(shooter::suck));
        addCommands(new InstantCommand(shooter::antiTransport));
    }
}