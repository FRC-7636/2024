package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.SINGLE_CMD.SmartIntake;
import frc.robot.commands.AUTO_CMD.AutoIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;

public class RightStart extends SequentialCommandGroup {

    public RightStart(Swerve m_swerve, Shooter shooter, Intake intake) {
       addCommands(new InstantCommand(intake::setFloorAngle));
       addCommands(new InstantCommand(() -> shooter.setPosition(50)));
       addCommands(new AutoShoot(shooter, intake));
//        Pose2d LLPose = LimelightHelpers.getBotPose2d_wpiBlue("");
//        if (LLPose.getX() != 0) {
//            addCommands(new InstantCommand(() -> m_swerve.resetOdometry(LLPose), m_swerve));
//        }
        // addCommands(new AutoIntake(shooter, intake)
                // .raceWith(m_swerve.getAutonomousCommand("blue 2 1", true)));
        addCommands(m_swerve.getAutonomousCommand("blue 2 2", false));
        // addCommands(new AutoShoot(shooter, intake));
    }
}
