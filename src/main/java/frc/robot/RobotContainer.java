// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.ejml.sparse.csc.decomposition.lu.LuUpLooking_FSCC;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PS5Controller.Axis;
// import edu.wpi.first.wpilibj.PS5Controller.;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AUTO_CMD.*;
import frc.robot.commands.GROUP_CMD.Amp;
import frc.robot.commands.GROUP_CMD.AmpShoot;
import frc.robot.commands.GROUP_CMD.AutoDriveToAmp;
import frc.robot.commands.GROUP_CMD.BackToOrigin;
import frc.robot.commands.GROUP_CMD.Reverse;
import frc.robot.commands.GROUP_CMD.SafeIntake;
import frc.robot.commands.SINGLE_CMD.ClimbUp;
import frc.robot.commands.SINGLE_CMD.ContinuousShoot;
import frc.robot.commands.SINGLE_CMD.FarShoot;
import frc.robot.commands.SINGLE_CMD.JustShoot;
import frc.robot.commands.SINGLE_CMD.NearShoot;
import frc.robot.commands.SINGLE_CMD.ShooterStandby;
import frc.robot.commands.SINGLE_CMD.ShooterSuck;
import frc.robot.commands.SINGLE_CMD.SmartIntake;
import frc.robot.commands.SINGLE_CMD.SmartShoot;
// import frc.robot.commands.SINGLE_CMD.StopAll;

import frc.robot.commands.SINGLE_CMD.TransportShoot;
import frc.robot.commands.SINGLE_CMD.TransportSuck;
import frc.robot.subsystems.*;
import frc.robot.commands.SWERVE_CMD.NewFieldDrive;
import frc.robot.commands.TEST_CMD.StopEverything;

public class RobotContainer {
    // Objects, Instances
    private final Swerve driveBase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/Neo"));
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();
    private final Candle candle = new Candle();
    private final Limelight limelight = new Limelight();
    // private final PhotonVision photonvison = new PhotonVision();
    private final XboxController chassisCtrl = new XboxController(0);
    private final XboxController assistCtrl = new XboxController(1);
    private final XboxController testCtrl = new XboxController(2);
    // private final PS5Controller Jay_Ctrl = new PS5Controller(3);
    private final XboxController Jay_Ctrl = new XboxController(3);

    // Auto Commands
    private final MiddleStart middleStart = new MiddleStart(driveBase, shooter, intake);
    private final LeftStart leftStart = new LeftStart(driveBase);
    private final RightStart rightStart = new RightStart(driveBase, shooter, intake);
    private final SendableChooser<Command> trajectoryChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // // TeleOp Commands
    // private final NewFieldDrive NFD = new NewFieldDrive(driveBase,
    //         chassisCtrl::getLeftY,
    //         chassisCtrl::getLeftX,
    //         chassisCtrl::getRightX);

    // private final NewFieldDrive chassisFD = new NewFieldDrive(driveBase,
    //         chassisCtrl::getLeftY,
    //         chassisCtrl::getLeftX,
    //         chassisCtrl::getRightX);

    private final NewFieldDrive chassisFD = new NewFieldDrive(driveBase, 
                Jay_Ctrl::getLeftY, 
                Jay_Ctrl::getLeftX, 
                Jay_Ctrl::getRightX);

    // private final NewFieldDrive chassisFD = new NewFieldDrive(driveBase,
    //             () -> PS5DeadbandLeftY(),
    //             () -> PS5DeadbandLeftX(),
    //             () -> PS5DeadbandRightX());

    private final StopEverything stopEverything = new StopEverything(intake, shooter);
    private final SmartShoot smartShoot = new SmartShoot(shooter, intake, driveBase, limelight);
    private final NearShoot nearShoot = new NearShoot(shooter, intake);
    private final ContinuousShoot continuousShoot = new ContinuousShoot(shooter, intake, climber);
    private final FarShoot farShoot = new FarShoot(shooter, intake);
    private final JustShoot justShoot = new JustShoot(shooter, intake);
    private final SmartIntake smartIntake = new SmartIntake(shooter, intake);
    private final BackToOrigin backToOrigin = new BackToOrigin(climber, intake, shooter);
    private final Amp ampCmd = new Amp(climber, shooter, intake);
    private final Reverse reverse = new Reverse(shooter, intake);
    private final AmpShoot ampShoot = new AmpShoot(shooter, intake);
    private final TransportSuck transportSuck = new TransportSuck(shooter, intake);
    private final TransportShoot transportShoot = new TransportShoot(shooter, intake);
    private final SafeIntake safeIntake = new SafeIntake(intake, shooter, climber);
    private final ClimbUp climbUp = new ClimbUp(climber);
    private final ShooterStandby shooterStandby = new ShooterStandby(shooter, intake);
    private final ShooterSuck shooterSuck = new ShooterSuck(shooter);
    // private final StopAll stopAll = new StopAll(intake, shooter, climber);
    // private final AutoDriveToAmp autoDriveToAmp = new AutoDriveToAmp(driveBase, limelight, null, intake, climber);
    
    private SendableChooser<Command> getAutoChooser = new SendableChooser<>();

    private final static File[] pathFileList = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();
    
    public RobotContainer() {
        // Configure controller buttons
        configureBangRenBindings();
        configureYuJieBindings();
        configureTestBindings();
        
        Jay_configureBindings();

        createButtonsOnDS();

        getAutoChooser = AutoBuilder.buildAutoChooser();
        
        // NamedCommands.registerCommand("amplevel", ampCmd);
        // NamedCommands.registerCommand("ampshoot", new InstantCommand(intake::shoot));
        //SmartDashboard.putData(getAutoChooser);
       // getAutoChooser.setDefaultOption("MiddleStart", leftStart);
       // getAutoChooser.addOption("LeftStart", middleStart);

       // SmartDashboard.putData("Auto", getAutoChooser);
        driveBase.setDefaultCommand(chassisFD);
    }
    
    private void configureBangRenBindings() {
        // new JoystickButton(chassisCtrl, 1).onTrue(new InstantCommand(() -> shooter.setPosition(50), shooter));
        // new JoystickButton(chassisCtrl, 2).onTrue(ampShoot);
        // new JoystickButton(chassisCtrl, 3).onTrue(reverse);
        // new JoystickButton(chassisCtrl, 4).onTrue(nearShoot);

        // new POVButton(chassisCtrl, 0).whileTrue(justShoot);
        // new POVButton(chassisCtrl, 180).onTrue(farShoot);

        // new JoystickButton(chassisCtrl, 5).whileTrue(new InstantCommand(intake::reverseConvey)).onFalse(new InstantCommand(intake::stopAll));
        // new JoystickButton(chassisCtrl, 6).whileTrue(new InstantCommand(intake::shoot)).onFalse(new InstantCommand(intake::stopAll));

        // new JoystickButton(chassisCtrl, 7).onTrue(new InstantCommand(driveBase::setIntakeAsHead));
    }
    private void configureYuJieBindings() {
        // new JoystickButton(assistCtrl, 1).onTrue(ampCmd);
        // new JoystickButton(assistCtrl, 2).onTrue(backToOrigin);
        // new JoystickButton(assistCtrl, 3).onTrue(smartIntake.andThen(transportSuck).andThen(transportShoot));
        // new JoystickButton(assistCtrl, 4).whileTrue(new InstantCommand(intake::suck, intake)).onFalse(new InstantCommand(intake::stopIntake));

        // new JoystickButton(assistCtrl, 5).whileTrue(new InstantCommand(intake::setFloorAngle)
        //         .alongWith(new InstantCommand(intake::slowSuck))
        //         .alongWith(new InstantCommand(intake::reverseConvey)))
        //         .onFalse(new InstantCommand(intake::stopAll));
        // new JoystickButton(assistCtrl, 6).whileTrue(new InstantCommand(intake::reverseConvey)).onFalse(new InstantCommand(intake::stopAll));

        // new POVButton(assistCtrl, 0).whileTrue(new InstantCommand(climber::up, climber)).onFalse(new InstantCommand(climber::stop));
        // new POVButton(assistCtrl, 180).whileTrue(new InstantCommand(climber::down, climber)).onFalse(new InstantCommand(climber::stop));
        // new POVButton(assistCtrl, 270).whileTrue(new InstantCommand(climber::setFloorLevel, climber));
    }
    
    private void configureTestBindings(){
        // new JoystickButton(testCtrl, 1).onTrue(new InstantCommand(intake::shoot).andThen(intake::setReverseAngle).andThen(shooter::intakeAngle).andThen(new WaitCommand(1)).andThen(reverse)); //reversing note from shooter to intake
        // new JoystickButton(testCtrl, 2).onTrue(backToOrigin); //back to origin
        // new JoystickButton(testCtrl, 3).onTrue(safeIntake);
        // new JoystickButton(testCtrl, 4).onTrue(smartShoot); //shootering
        // new JoystickButton(testCtrl, 5).whileTrue(new InstantCommand(intake::reverseConvey)).whileTrue(new InstantCommand(intake::shoot)).onFalse(new InstantCommand(intake::stopAll)); //reverse convey
        // new JoystickButton(testCtrl, 6).whileTrue(new InstantCommand(intake::shoot)).onFalse(new InstantCommand(intake::stopAll)); //intake shoot
        // new JoystickButton(testCtrl, 7).onTrue(new InstantCommand(driveBase::setIntakeAsHead)); //set intake as head

        // new POVButton(testCtrl, 0).onTrue(ampCmd); //amp
        // new POVButton(testCtrl, 90).onTrue(ampShoot); //amp shoot
        // new POVButton(testCtrl, 180).onTrue(continuousShoot); //near shoot
        // new POVButton(testCtrl, 270).whileTrue(new InstantCommand(intake::suck, intake)).onFalse(new InstantCommand(intake::stopIntake)); //intake suck
    }

    public void Jay_configureBindings(){
        new JoystickButton(Jay_Ctrl, 1).whileTrue(new InstantCommand(shooter::shootslow)).onFalse(new InstantCommand(shooter::stopAll));
        new JoystickButton(Jay_Ctrl, 2).onTrue(backToOrigin); //back to origin
        new JoystickButton(Jay_Ctrl, 4).whileTrue(reverse).onFalse(new InstantCommand(intake::stopAll)).onFalse(new InstantCommand(shooter::stopAll));
        new JoystickButton(Jay_Ctrl, 5).whileTrue(ampShoot).onFalse(backToOrigin).onFalse(new InstantCommand(intake::stopAll)); //AmpShoot
        new JoystickButton(Jay_Ctrl, 6).whileTrue(shooterSuck).onFalse(new InstantCommand(shooter::stopAll));
        new JoystickButton(Jay_Ctrl, 7).whileTrue(new InstantCommand(shooter::shoot)).onFalse(new InstantCommand(shooter::stopAll)); //intake shoot       
        new JoystickButton(Jay_Ctrl, 8).whileTrue(justShoot).onFalse(new InstantCommand(shooter::stopAll)).onFalse(new InstantCommand(intake::stopAll)); //intake set floor angle
        
        new POVButton(Jay_Ctrl, 0).onTrue(new InstantCommand(climber::setBalanceLevel, climber)); // climber balance level
        // new POVButton(Jay_Ctrl, 180).whileTrue(new InstantCommand(climber::down, climber)).onFalse(new InstantCommand(climber::stop)); //climber down
        // new POVButton(Jay_Ctrl, 90).whileTrue(climbUp);

        new JoystickButton(chassisCtrl, 1).onTrue(backToOrigin);
        new JoystickButton(chassisCtrl, 2).whileTrue(safeIntake).onFalse(new InstantCommand(intake::stopAll)).onFalse(new InstantCommand(shooter::stopAll)).onFalse(backToOrigin);
        new JoystickButton(chassisCtrl, 3).whileTrue(new InstantCommand(intake::reverseConvey)).onFalse(new InstantCommand(intake::stopAll)); //Amp
        new JoystickButton(chassisCtrl, 4).onTrue(new InstantCommand(intake::setFloorAngle)).whileTrue(new InstantCommand(intake::suck)).onFalse(new InstantCommand(intake::stopIntake)); // intake suck
        new JoystickButton(chassisCtrl, 5).whileTrue(new InstantCommand(intake::suck)).onFalse(new InstantCommand(intake::stopIntake)); // intake suck
        new JoystickButton(chassisCtrl, 6).onTrue(new InstantCommand(intake::setAmpAngle)).whileTrue(new InstantCommand(intake::shoot)).onFalse(new InstantCommand(intake::stopAll)); //intake shoot
        new JoystickButton(chassisCtrl, 7).onTrue(climbUp).whileFalse(stopEverything);
        
        new POVButton(chassisCtrl, 0).onTrue(ampCmd); //amp
        new POVButton(chassisCtrl, 90).whileTrue(new InstantCommand(climber::up, climber)).onFalse(new InstantCommand(climber::stop)); //climber down
        new POVButton(chassisCtrl, 180).whileTrue(new InstantCommand(climber::down, climber)).onFalse(new InstantCommand(climber::stop)); //climber down
        new POVButton(chassisCtrl, 270).whileTrue(new InstantCommand(climber::setFloorLevel, climber)); //climber set floor level
    }

    public void createButtonsOnDS() {
        // SmartDashboard.putData("AMP Stage", new Amp(climber, shooter, intake));
        // SmartDashboard.putData("AMP Stage", new Amp(climber, shooter, intake));
        // SmartDashboard.putData("SafeIntake", new SafeIntake(intake, shooter));
        // SmartDashboard.putData("Smart Shoot", new SmartShoot(shooter, intake, driveBase, limelight));
        // SmartDashboard.putData("Rervse Note", new InstantCommand(intake::shoot).andThen(intake::setReverseAngle).andThen(shooter::intakeAngle).andThen(new WaitCommand(1)).andThen(new Reverse(shooter, intake)));
        // SmartDashboard.putData("Back to Origin", new BackToOrigin(climber, intake, shooter));
        // SmartDashboard.putData("Back to Origin", new BackToOrigin(climber, intake, shooter));
        // SmartDashboard.putData("AmpShoot", new AmpShoot(shooter, intake));
        // SmartDashboard.putData("JustShoot", new JustShoot(shooter, intake));
        // SmartDashboard.putData("TransportShoot", new TransportShoot(shooter, intake));
        // SmartDashboard.putData("NearShoot", new NearShoot(shooter, intake));
        // SmartDashboard.putData("SmartIntake", new SmartIntake(shooter, intake).andThen(justShoot));
        // SmartDashboard.putData("Auto AMP Shoot", new AutoDriveToAmp(driveBase, limelight, null, intake, climber));
        // SmartDashboard.putData("Auto AMP Shoot", new AutoDriveToAmp(driveBase, limelight, null, intake));
        // SmartDashboard.putData("Floorangle", new InstantCommand(intake::setFloorAngle, intake));
        // SmartDashboard.putData("AmpAngle", new InstantCommand(intake::setAmpAngle));
        // SmartDashboard.putData("ShooterSuck", new ShooterSuck(shooter));
        // SmartDashboard.putData("Shoot", new ShooterStandby(shooter, intake));
    }

    private Command wait(double d) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'wait'");
    }

    public Command getAutonomousCommand() {
        // return new BlueTest1(driveBase,shooter);
        // return new LeftStart(photonVision, driveBase);
        // return new MiddleStart(driveBase);
        // return new RightStart(driveBase);
        return null;

        // return trajectoryChooser.getSelected();
        // return getAutoChooser.getSelected();
        // return new AutoAmp(driveBase, intake, climber, shooter);
    }
    
    public void initiateTrajectoryChooser() {
        // Initiate SendableChooser
        List<String> trajectoryList = new ArrayList<>();
        System.out.println("Detected paths:");
        for (File file : Objects.requireNonNull(pathFileList)) {
            if (file.isFile()) {
                String pureFileName = file.getName().replaceFirst("[.][^.]+$", "");  // Use regex to remove extension
                trajectoryList.add(pureFileName);
                System.out.println(" " + pureFileName);
            }
        }
        trajectoryChooser.setDefaultOption("Run all", middleStart);
        for (String name : trajectoryList) {
            trajectoryChooser.addOption(name, new SingleTrajectory(driveBase, name));
        }
        SmartDashboard.putData("Choose Trajectory", trajectoryChooser);
    }

    public void initiateAutoChooser() {
        autoChooser.setDefaultOption("MiddleStart", leftStart);
        autoChooser.addOption("LeftStart", middleStart);

        SmartDashboard.putData("Choose Auto", autoChooser);
    }
    
    /*
    public void rumbleWhenReady() {
        if (shooter.readyToShoot()) {
            chassisCtrl.setRumble(RumbleType.kBothRumble, 0.5);
        }
        else {
            chassisCtrl.setRumble(RumbleType.kBothRumble, 0);
        }
    }   
    */
    public double PS5DeadbandRightX(){
        double deadband = 0.02;
        double RightX = Jay_Ctrl.getRightX();
        
        if(RightX > 0){ 
            if(RightX < deadband){
                RightX = 0;
            }
        }
        else if(RightX < 0){
            if(RightX > deadband){
                RightX = 0;
            }
        }
        return RightX;
        
    }
    public double PS5DeadbandLeftX(){
        double deadband = 0.02;
        double LeftX = Jay_Ctrl.getLeftX();

        if(LeftX > 0){  
            if(LeftX < deadband){
                LeftX = 0;
            }
        }
        else if(LeftX < 0){  
            if(LeftX > deadband){
                LeftX = 0;
            }
        }
        return LeftX;
    }

    public double PS5DeadbandLeftY(){
        double deadband = 0.03;
        double LeftY = Jay_Ctrl.getLeftY();

        if(LeftY > 0){   
            if(LeftY < deadband){
                LeftY = 0;
            }
        }
        else if(LeftY < 0){ 
            if(LeftY > deadband){
                LeftY = 0;
            }
        }
        return LeftY;
    }
}