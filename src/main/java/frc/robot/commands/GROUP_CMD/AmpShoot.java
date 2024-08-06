package frc.robot.commands.GROUP_CMD;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;


public class AmpShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;

    public AmpShoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        // addRequirements(this.shooter, this.intake);
        addRequirements(this.shooter, this.intake);
    }

    @Override
    public void execute() {
        // shooter.AmpAngle();
        // shooter.AMPshoot();
        // shooter.transport();
        intake.shoot();
        intake.setAmpAngle();
        // if (!shooter.noteDetected()){
        //     shooter.originAngle();
        //     shooter.stopAll();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        intake.stopConvey();
    }
}
