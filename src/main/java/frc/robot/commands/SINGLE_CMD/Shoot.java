package frc.robot.commands.SINGLE_CMD;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command{
    private final Shooter shooter;
    private final Intake intake;

    public Shoot(Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooter, this.intake);
    }
    
    public void execute(){
        if(shooter.noteDetected()){
            shooter.shoot();
            shooter.antiTransport();
            shooter.setPosition(ShooterConstants.Control.NEARSHOOT_POSITION);
            intake.setFloorAngle();
        }
        else if(shooter.noteDetected() && shooter.velocityReady()){
            shooter.shoot();
            shooter.ShooterTransport();
            shooter.setPosition(ShooterConstants.Control.NEARSHOOT_POSITION);
            intake.startConvey();
            intake.backToZero();
        }
        else if(!shooter.noteDetected()){
            shooter.stopAll();
            shooter.setPosition(ShooterConstants.Control.ORIGIN_POSITION);
            intake.stopAll();
            intake.backToZero();
        }
        
    }
    public boolean isFinished(){
        return !shooter.noteDetected();
    }

    public void end(boolean interrupted){
        shooter.stopAll();
        shooter.setPosition(ShooterConstants.Control.ORIGIN_POSITION);
        intake.stopAll();
        intake.backToZero();
    }
}
