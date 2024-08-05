// package frc.robot.commands.TEST_CMD;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// import frc.robot.subsystems.*;
// import frc.robot.LimelightHelpers;
// import frc.robot.commands.GROUP_CMD.Amp;
// import frc.robot.Constants;

// public class AutoAmpPos extends Command{
//     private final Swerve swerve;
//     private final Limelight limelight;
//     private final Shooter shooter;
//     private final Climber climber;
//     private final Intake intake;
//     private boolean stop;
//    // private PIDController driveCtrl = new PIDController(Constants.AutoDrivePIDF.P, Constants.AutoDrivePIDF.I, Constants.AutoDrivePIDF.D);
//     private PIDController turnCtrl = new PIDController(Constants.AutoTurnPIDF.P, Constants.AutoTurnPIDF.I, Constants.AutoTurnPIDF.D);
//     public AutoAmpPos(Swerve swerve, Limelight limelight, Shooter shooter, Intake intake, Climber climber) {
//         this.swerve = swerve;
//         this.limelight = limelight;
//         this.climber = climber;
//         this.intake = intake;
//         this.shooter = shooter;
//         addRequirements(this.swerve, this.limelight, this.climber, this.intake, this.shooter);
//     }

//     @Override
//     public void execute(){
//         Pose2d LLPose = LimelightHelpers.getBotPose2d("");
//         if (LLPose.getX() != 0) {
//            swerve.resetOdometry(LLPose);
//            stop = false;
//         }
//         else {
//             stop = true; 
//         }
//         Rotation2d ampRotation2d = new Rotation2d(Math.toRadians(90));
//         Pose2d swervePos = swerve.getPose();
//         Pose2d ampPose2d = new Pose2d(1.88, 7.78, ampRotation2d);
//         Transform2d deltaTransform2d = swervePos.minus(ampPose2d);
//         Translation2d targetTranslation2d = deltaTransform2d.getTranslation();
//         Rotation2d targetRotation2d = deltaTransform2d.getRotation();
//         double deltaDeg = targetRotation2d.getDegrees();
//         if ((Math.abs(deltaTransform2d.getX()) < 3) && (Math.abs(deltaTransform2d.getY()) < 3) && (Math.abs(deltaDeg) < 3)){
//             swerve.drive(targetTranslation2d, turnCtrl.calculate(deltaDeg), false);
//             stop = true;
//         }
//         else {
//             swerve.drive(new Translation2d(), 0, false);
//         }
//     }

//     // @Override
//     // public boolean isFinished(){
//     //     return stop;
//     // }

//     // @Override
//     // public void end(boolean interrupted) {
//     // }
// }