package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class BlueTest1 extends SequentialCommandGroup {
    public BlueTest1(Swerve swerve) {
        addCommands(Commands.runOnce(() -> swerve.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
        addCommands(swerve.getAutonomousCommand("blue 1 test1", false));
        addCommands(new WaitCommand(.5));
        addCommands(swerve.getAutonomousCommand("blue 1 test2", false));
    }
}