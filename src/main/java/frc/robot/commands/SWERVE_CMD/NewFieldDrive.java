package frc.robot.commands.SWERVE_CMD;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import swervelib.math.SwerveMath;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;


public class NewFieldDrive extends Command {
    private final Swerve swerve;
    private final DoubleSupplier vX, vY, heading;

    public NewFieldDrive(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;
        addRequirements(swerve);    
    }

    @Override
    public void execute() {
        // if (vX.getAsDouble() == 0 & vY.getAsDouble() == 0 & heading.getAsDouble() == 0) {
        //     SmartDashboard.putNumber("X", 0);
        //     SmartDashboard.putNumber("Y", 0);
        //     Timer.delay(0.1);
        //     swerve.lock();
        // }
        // else {
            Translation2d translation = new Translation2d(-vX.getAsDouble()*6, -vY.getAsDouble()*6);
            translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                                   Constants.Chassis.LOOP_TIME, Constants.Chassis.ROBOT_MASS, List.of(Constants.Chassis.CHASSIS),
                                                   swerve.getSwerveDriveConfiguration());
            SmartDashboard.putNumber("X", translation.getX());
            SmartDashboard.putNumber("Y", translation.getY());
        
            swerve.drive(translation, -heading.getAsDouble()*8, true);
        // }
    }
}
