package frc.robot.commands.SWERVE_CMD;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;


public class AbsDrive extends Command {
  private final Swerve swerve;
  private final DoubleSupplier vX, vY;
  private final DoubleSupplier heading;

  public AbsDrive(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    addRequirements(swerve);
  }


  @Override
  public void execute()
  {
    new Rotation2d();
    Rotation2d heading2d = Rotation2d.fromRadians(-heading.getAsDouble()*Math.PI);
    SmartDashboard.putNumber("Desired Heading", heading2d.getDegrees());
    // Get the desired chassis speeds based on a 2 joystick module.
    // set current joystick position as "desired heading"
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(-vX.getAsDouble(), -vY.getAsDouble(), heading2d);

    // Prevent Movement After Auto
    
    // if(initRotation)
    // {
    //   if(heading.getAsDouble() == 0)
    //   {
    //     // Get the curretHeading
    //     Rotation2d firstLoopHeading = swerve.getHeading();
    //     // Set the Current Heading to the desired Heading
    //     desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
    //   }
    //   //Dont Init Rotation Again
    //   initRotation = false;
    // }

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                           Constants.Chassis.LOOP_TIME, Constants.Chassis.ROBOT_MASS, List.of(Constants.Chassis.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.drive(translation, -heading.getAsDouble()*5, true);
  }
}
