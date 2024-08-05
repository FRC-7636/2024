package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.LimelightHelpers;
import frc.robot.commands.SINGLE_CMD.SmartShoot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.SINGLE_CMD.SmartIntake;

public class MiddleStart extends SequentialCommandGroup {

    // public MiddleStart(Swerve m_swerve, Shooter shooter, Intake intake) {
    public MiddleStart(Swerve m_swerve) {
        // addCommands(new InstantCommand(intake::setFloorAngle));
        // addCommands(new InstantCommand(() -> shooter.setPosition(50)));
        // addCommands(new SmartShoot(shooter, intake, m_swerve));
        addCommands(m_swerve.getAutonomousCommand("simple go out", true));
//        Pose2d LLPose = LimelightHelpers.getBotPose2d_wpiBlue("");
//        if (LLPose.getX() != 0) {
//            addCommands(new InstantCommand(() -> m_swerve.resetOdometry(LLPose), m_swerve));
//        }
//        addCommands(new AutoIntake(shooter, intake)
//                .raceWith(m_swerve.getAutonomousCommand("mid 1", true)));
//        addCommands(new AutoShoot(shooter, intake));
//        addCommands(new AutoIntake(shooter, intake)
//                .raceWith(m_swerve.getAutonomousCommand("mid 3 3", false)));
//        addCommands(new AutoShoot(shooter, intake));
//        addCommands(new AutoIntake(shooter, intake)
//                .raceWith(m_swerve.getAutonomousCommand("mid 3 2", false)));
//        addCommands(new AutoShoot(shooter, intake));
    }
}
