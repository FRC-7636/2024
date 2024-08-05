package frc.robot.commands.GROUP_CMD;

import javax.print.attribute.standard.MultipleDocumentHandling;
import java.util.Timer;
import java.util.TimerTask;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.SINGLE_CMD.SmartIntake;
import frc.robot.commands.SINGLE_CMD.TransportShoot;
import frc.robot.commands.SINGLE_CMD.TransportSuck;
import frc.robot.commands.TEST_CMD.StopEverything;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class SafeIntake extends SequentialCommandGroup{
    // private double timeOut = 1000;
    // private Timer timer = new Timer();
    // private TimerTask timeOutTask;
    // private boolean isTimeOut = false;
    public SafeIntake(Intake intake,Shooter shooter){
        // timer.schedule(timeOutTask, (long) timeOut);
        // timeOutTask = new TimerTask() {
        //     public void run(){
        //         isTimeOut = true;
        //     }
        // };
        addCommands(new SmartIntake(shooter, intake));
        addCommands(new TransportSuck(shooter, intake));
        addCommands(new TransportShoot(shooter, intake));
        addCommands(new InstantCommand(shooter::standby));
        // if (shooter.currentVel() >10) {
        //     isTimeOut = true;
        // }
        // if (isTimeOut){
            //addCommands(new StopEverything(intake, shooter));
        // }
        // addCommands(new InstantCommand(shooter::transport).
        //                 raceWith(new InstantCommand(intake::reverseConvey)).
        //                 andThen(new WaitCommand(.5)).
        //                 andThen(shooter::stopAll).andThen(shooter::standby));
    }
}