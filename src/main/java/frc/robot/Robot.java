// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final Field2d field2d = new Field2d();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

//    m_robotContainer.initiateTrajectoryChooser();
//    m_robotContainer.initiateAutoChooser();

//    CameraServer.startAutomaticCapture();

    var allianceSide = DriverStation.getAlliance();
    if (allianceSide.isPresent()) {
      if (allianceSide.get() == DriverStation.Alliance.Red) {
        LimelightHelpers.setPriorityTagID("", 4);
      }
      else {
        LimelightHelpers.setPriorityTagID("", 7);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    field2d.setRobotPose(LimelightHelpers.getBotPose2d_wpiBlue(""));
    SmartDashboard.putData("LL Field", field2d);
  }


  @Override
  public void disabledPeriodic() {}


  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
//    m_robotContainer.autoResetOdometry();

    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {}


  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}
}
