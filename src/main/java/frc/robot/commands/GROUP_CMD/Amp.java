package frc.robot.commands.GROUP_CMD;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;


public class Amp extends Command {
    private final Climber climber;
    private final Shooter shooter;
    private final Intake intake;

    public Amp(Climber climber, Shooter shooter, Intake intake) {
        this.climber = climber;
        this.shooter = shooter;
        this.intake = intake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climber, this.shooter, this.intake);
    }

    @Override
    public void execute() {
        shooter.originAngle();
        climber.setBalanceLevel();
        intake.setAmpAngle();
    }
}
