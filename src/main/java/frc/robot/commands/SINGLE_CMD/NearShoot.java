package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

public class NearShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;

    public NearShoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(this.shooter, this.intake);
    }


    @Override
    public void execute() {
        shooter.setPosition(ShooterConstants.Control.NEARSHOOT_POSITION);
        shooter.shootTwo();
        shooter.ShooterTransport();
        if (shooter.readyToShoot()) {
            intake.conveyorShoot();
            shooter.transport();
            shooter.shootNear();
        }
    }

    @Override
    public boolean isFinished() {
        return !shooter.noteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        intake.stopConvey();
    }
}
