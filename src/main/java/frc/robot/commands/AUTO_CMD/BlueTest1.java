package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SINGLE_CMD.*;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;

public class BlueTest1 extends SequentialCommandGroup {
    public BlueTest1(Swerve swerve, Shooter shooter) {
        //addCommands(Commands.runOnce(() -> swerve.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
        //4erre4qwrraddCommands(swerve.getAutonomousCommand("blue 1 test1", true));
       // addCommands(new WaitCommand(.5));
        //addCommands(swerve.getAutonomousCommand("blue 1 test2", false));
        // addCommands(new InstantCommand(shooter::shoot));
        // addCommands(new InstantCommand(shooter::NearShootAngle));
        // addCommands(new WaitCommand(5));
        // addCommands(new InstantCommand(shooter::transport));
        addCommands(new NearShoot(shooter, null));
        // addCommands(swerve.getAutonomousCommand("blue 1 test 1", true));

    }
}