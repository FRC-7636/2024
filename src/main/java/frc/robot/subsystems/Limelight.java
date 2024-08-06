package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Limelight extends SubsystemBase {
    // private final Field2d field2d;
    private Swerve swerve;
    NetworkTable table = NetworkTableInstance.getDefault().getTable(VisionConstants.NAME);
    DoubleArraySubscriber CameraSeeBot = table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[]{});
    public Object robotToTarget;

    // public Limelight() {
        // field2d = new Field2d();
    // }

    @Override
    public void periodic() {
       // field2d.setRobotPose(LimelightHelpers.getBotPose2d_wpiBlue(""));
       // SmartDashboard.putData("Lime_Field2d", field2d);
        SmartDashboard.putNumber("deltaTargetDeg", deltaTargetDeg());
        SmartDashboard.putNumber("deltaRobotHeadingDeg", deltaRobotHeadingDeg());
    //     Rotation2d ampRotation2d = new Rotation2d(Math.toRadians(90));
    //     Pose2d swervePos = swerve.getPose();
    //     Pose2d ampPose2d = new Pose2d(1.88, 7.78, ampRotation2d);
    //     Transform2d deltaTransform2d = swervePos.minus(ampPose2d);
    //     Translation2d targetTranslation2d = deltaTransform2d.getTranslation();
    //     Rotation2d targetRotation2d = deltaTransform2d.getRotation();
    //     double deltaDeg = targetRotation2d.getDegrees();
    // SmartDashboard.putNumber("deltaDeg", deltaDeg);
    }

    public Pose3d robotToTarget(){
        return LimelightHelpers.getBotPose3d_TargetSpace("");
    }
    
    public double deltaTargetDeg(){
        double Deg = Math.toDegrees(Math.atan2(robotToTarget().getX(), robotToTarget().getZ()));
            if (Deg > 0) {
                Deg = 180 - Deg;
            }
            else if(Deg < 0){
                Deg = -180 - Deg;
            }
            else if(Deg == 0){
                Deg = 0;
            }
            return Deg;
            }

    public double deltaRobotHeadingDeg(){
        double heading = LimelightHelpers.getBotPose2d("").getRotation().getDegrees();
        double ID = LimelightHelpers.getFiducialID("");
        if(ID == 3 || ID == 4){
            System.out.println("ID34");
            return -heading;
        }
        else if(ID == 7 || ID == 8){
            System.out.println("ID78s");
            if (heading > 0) {
                heading = 180 - heading;
            }
            else if(heading < 0){
                heading = -180 - heading;
            }
            else if(heading == 0){
                heading = 0;
            }
            return heading;
            }
        return heading;
    }
    
    public void blink(){
        LimelightHelpers.setLEDMode_ForceBlink("");
    }

    public void LEDon(){
        LimelightHelpers.setLEDMode_ForceOn("");
    }

    public void LEDoff(){
        LimelightHelpers.setLEDMode_ForceOff("");
    }

    public void LEDpipe(){
        LimelightHelpers.setLEDMode_PipelineControl("");
    }
    
    public Pose2d getPose2d() {
        return LimelightHelpers.getBotPose2d_wpiBlue("");
    }


}