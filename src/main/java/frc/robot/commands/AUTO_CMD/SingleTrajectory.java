package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;

public class SingleTrajectory extends SequentialCommandGroup {
    public SingleTrajectory(PhotonVision m_photonVision, Swerve m_swerve, String pathName) {
        addCommands(Commands.runOnce(() -> m_swerve.resetOdometry(m_photonVision.getLatestEstimatedRobotPose()), m_swerve, m_photonVision));
        addCommands(m_swerve.getAutonomousCommand(pathName, false));
    }

    public SingleTrajectory(Swerve m_swerve, String pathName) {
//        addCommands(Commands.runOnce(() -> m_swerve.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("")), m_swerve));
        addCommands(m_swerve.getAutonomousCommand(pathName, true));
    }
}