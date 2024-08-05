package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;

public class LeftStart extends SequentialCommandGroup{
    public LeftStart(PhotonVision m_photonVision, Swerve m_swerve){
        addCommands(Commands.runOnce(() -> m_swerve.resetOdometry(m_photonVision.getLatestEstimatedRobotPose()), m_swerve, m_photonVision));
        addCommands(m_swerve.getAutonomousCommand("1", false));
        addCommands(m_swerve.getAutonomousCommand("2", false));
        addCommands(m_swerve.getAutonomousCommand("2 Shoot", false));
    }

    public LeftStart(Swerve m_swerve){
        addCommands(Commands.runOnce(() -> m_swerve.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("")), m_swerve));
        addCommands(m_swerve.getAutonomousCommand("1", false));
        addCommands(m_swerve.getAutonomousCommand("2", false));
        addCommands(m_swerve.getAutonomousCommand("2 Shoot", false));
    }
}