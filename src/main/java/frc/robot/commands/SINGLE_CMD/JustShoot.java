package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class JustShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;

    public JustShoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooter, this.intake);
    }

    @Override
    public void execute() {
        shooter.setPosition(ShooterConstants.Control.NEARSHOOT_POSITION);
        shooter.shoot();
        intake.backToZero();
        shooter.ShooterTransport();
        intake.startConvey();
        }
    

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        intake.stopConvey();
    }
}
