package frc.robot.commands.SWERVE_CMD;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.subsystems.Swerve;


public class FieldRelativeDrive extends Command {
    private final Swerve swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier heading;

    public FieldRelativeDrive(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        new Rotation2d();
        Rotation2d heading2d = Rotation2d.fromRotations(-heading.getAsDouble()*0.499);
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(-vX.getAsDouble(), -vY.getAsDouble(), heading2d);
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, swerve.getHeading());

        swerve.drive(fieldSpeeds);
    }
}