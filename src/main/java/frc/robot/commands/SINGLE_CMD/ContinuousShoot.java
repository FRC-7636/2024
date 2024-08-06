package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ShooterConstants;

public class ContinuousShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private final Climber climber;

    public ContinuousShoot(Shooter shooter, Intake intake, Climber climber) {
        this.shooter = shooter;
        this.intake = intake;
        this.climber = climber;
        addRequirements(this.shooter, this.intake, this.climber);
    }


    @Override
    public void execute() {
        climber.setPosition(3.5);
        intake.setFloorAngle();
        intake.suck();
        intake.startConvey();
        shooter.ShooterTransport();
        Pose3d robotToTarget = LimelightHelpers.getBotPose3d_TargetSpace("");
        double deltaDegY = 90 - Math.toDegrees(Math.atan2(-robotToTarget.getZ(), robotToTarget.getY() + 0.53));
        System.out.println(deltaDegY - ShooterConstants.Config.ANGLE_OFFSET);
        shooter.setPosition(MathUtil.clamp(deltaDegY - ShooterConstants.Config.ANGLE_OFFSET, 5, 45));
        shooter.shoot();
    }
}
