package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class TransportShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;

    public TransportShoot (Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(this.shooter);
        addRequirements(this.shooter, this.intake);
    }

    public void execute(){
        intake.startConvey();
        shooter.transport();
        intake.safeIntake();
    }

    @Override
    public boolean isFinished(){
        return shooter.noteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        intake.stopAll();
    } 
}