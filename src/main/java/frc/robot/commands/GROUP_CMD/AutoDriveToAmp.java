
package frc.robot.commands.GROUP_CMD;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;

public class AutoDriveToAmp extends SequentialCommandGroup{
    public AutoDriveToAmp(Swerve swerve, Limelight limelight, Shooter shooter, Intake intake, Climber climber) {
        Pose2d LLPose = LimelightHelpers.getBotPose2d_wpiBlue("");
        if (LLPose.getX() != 0) {
            addCommands(Commands.runOnce(() -> swerve.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
            addCommands(swerve.getAutonomousCommand("AmpAuto", false));
            addCommands(new Amp(climber, shooter, intake));
        }
        else{
            addCommands(null);
        } 
        
    }  
}
