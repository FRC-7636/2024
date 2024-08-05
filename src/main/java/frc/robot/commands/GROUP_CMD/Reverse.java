package frc.robot.commands.GROUP_CMD;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class Reverse extends Command {
        private final Shooter shooter;
        private final Intake intake;

        public Reverse(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
                
        addRequirements(this.shooter, this.intake);
        }

        @Override
        public void execute(){
                intake.setFloorAngle();
                intake.shoot();
                intake.setReverseAngle();
                intake.reverseConvey();
                shooter.setPosition(Constants.ShooterConstants.Control.INTAKE_POSITION);
                shooter.antiTransport();
                shooter.suck();
        }

        @Override
        public boolean isFinished(){
                return (Math.abs(intake.intakeVel()) < 5300) && !shooter.noteDetected();
        }
        
        @Override
        public void end(boolean interrupted) {
        shooter.stopAll();
        intake.stopAll();
        // intake.stopConvey();
        // intake.stopIntake();
        shooter.topAngle();
        intake.backToZero();
        }
}




// addCommands();
//         addCommands(new InstantCommand(intake::setFloorAngle));
//         addCommands(new InstantCommand(() -> shooter.setPosition(Constants.ShooterConstants.Control.INTAKE_POSITION)).
//                 andThen(shooter::antiTransport));
//         addCommands(new InstantCommand(intake::setReverseAngle).
//                 andThen(intake::reverseConvey));
//         addCommands(new InstantCommand(intake::shoot));
//         addCommands(new WaitCommand(1).
//                 andThen(intake::stopAll).
//                 andThen(shooter::stopAll).
//                 andThen(intake::backToZero).
//                 andThen(shooter::originAngle));