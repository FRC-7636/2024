package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

public class AutoIntake extends Command {
    private final Intake intake;
    private final Shooter shooter;

    public AutoIntake(Shooter shooter, Intake intake) {
        this.intake = intake;
        this.shooter = shooter;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intake, this.shooter);
    }


    @Override
    public void execute() {
        shooter.setPosition(ShooterConstants.Control.INTAKE_POSITION);
        intake.setFloorAngle();
        intake.suck();
        intake.startConvey();
        shooter.transport();
    }

    @Override
    public boolean isFinished() {
        return shooter.noteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopAll();
    }
}
