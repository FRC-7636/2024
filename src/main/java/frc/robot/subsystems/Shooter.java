package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.LimelightHelpers;

public class Shooter extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.Config.L_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.Config.R_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final DigitalInput noteSensor = new DigitalInput(ShooterConstants.Config.SENSOR_ID);
    private final CANSparkMax transMotor = new CANSparkMax(ShooterConstants.Config.TRANS_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax angleMotor = new CANSparkMax(ShooterConstants.Config.ANGLE_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final SparkPIDController anglePIDController, leftPIDController, rightPIDController;
    private final Limelight limelight = new Limelight();

    private final RelativeEncoder leftMotorEncoder, rightMotorEncoder;
    private final AbsoluteEncoder angleEncoder;

    private double desiredSpeed = 0;
    private double desiredAngle = 0;

    /**
     * set PID using SparkPIDController
     * @param sparkPIDController sparkPIDController which you want to set PID for
     * @param p set P
     * @param i set I
     * @param d set D
     */
    private void setPID(SparkPIDController sparkPIDController, double p, double i, double d) {
        sparkPIDController.setP(p);
        sparkPIDController.setI(i);
        sparkPIDController.setD(d);
    }


    public Shooter() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        transMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        transMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setInverted(ShooterConstants.Config.SHOOTER_INVERTED);
        rightMotor.setInverted(ShooterConstants.Config.SHOOTER_INVERTED);
        transMotor.setInverted(ShooterConstants.Config.TRANS_INVERTED);
        angleMotor.setInverted(ShooterConstants.Config.ANGLE_INVERTED);

        leftMotor.setSmartCurrentLimit(ShooterConstants.Config.CURRENT_LIMIT);
        rightMotor.setSmartCurrentLimit(ShooterConstants.Config.CURRENT_LIMIT);
        transMotor.setSmartCurrentLimit(ShooterConstants.Config.CURRENT_LIMIT);
        angleMotor.setSmartCurrentLimit(ShooterConstants.Config.CURRENT_LIMIT);

        leftMotorEncoder = leftMotor.getEncoder();
        rightMotorEncoder = rightMotor.getEncoder();

        angleEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        angleEncoder.setPositionConversionFactor(360);

        // apply PIDs to motors
        anglePIDController = angleMotor.getPIDController();
        setPID(anglePIDController, ShooterConstants.AnglePIDF.P, ShooterConstants.AnglePIDF.I, ShooterConstants.AnglePIDF.D);
        leftPIDController = leftMotor.getPIDController();
        setPID(leftPIDController, ShooterConstants.LeftPIDF.P, ShooterConstants.LeftPIDF.I, ShooterConstants.LeftPIDF.D);
        rightPIDController = rightMotor.getPIDController();
        setPID(rightPIDController, ShooterConstants.RightPIDF.P, ShooterConstants.RightPIDF.I, ShooterConstants.RightPIDF.D);

        anglePIDController.setFeedbackDevice(angleEncoder);
        leftPIDController.setFeedbackDevice(leftMotorEncoder);
        rightPIDController.setFeedbackDevice(rightMotorEncoder);
        anglePIDController.setOutputRange(-0.35, 0.65);
        leftPIDController.setOutputRange(-1, 1);
        rightPIDController.setOutputRange(-1, 1);
        /* enable PID wrapping
         * after wrapping:
         *   set "0 -> 350", motor will go -10 instead of +350
         */
        anglePIDController.setPositionPIDWrappingEnabled(true);
        anglePIDController.setPositionPIDWrappingMinInput(0);
        anglePIDController.setPositionPIDWrappingMaxInput(360);

        leftMotor.burnFlash();
        rightMotor.burnFlash();
        transMotor.burnFlash();
        angleMotor.burnFlash();

        // anglePIDController.setReference(ShooterConstants.Control.TOP_POSITION, ControlType.kPosition);
        anglePIDController.setReference(ShooterConstants.Control.ORIGIN_POSITION, ControlType.kPosition);
    }

    /**
     * Set both shooter motors to the "shoot" status.
     * <p>(set the speed in {@link ShooterConstants.Control})
     */
    public void shoot() {
//         leftPIDController.setReference(ShooterConstants.Control.SHOOT_VELOCITY, ControlType.kVelocity);
//         rightPIDController.setReference(ShooterConstants.Control.SHOOT_VELOCITY, ControlType.kVelocity);
        desiredSpeed = ShooterConstants.Control.SHOOT_VELOCITY;
        leftMotor.set(0.6);
        rightMotor.set(0.6);
    }
    public void AMPshoot(){
        desiredSpeed = ShooterConstants.Control.SHOOT_VELOCITY;
        leftMotor.set(0.3);
        rightMotor.set(0.3);
    }
    public void shootNear() {
        desiredSpeed = ShooterConstants.Control.SHOOT_NEAR_VELOCITY;
        leftMotor.set(0.7);
        rightMotor.set(0.7);
    }

    /**
     * Set both shooter motors to the "suck" status.
     * <p>(set the speed in {@link ShooterConstants.Control})
     */
    public void suck() {
        leftMotor.set(ShooterConstants.Control.SUCK_SPEED);
        rightMotor.set(ShooterConstants.Control.SUCK_SPEED);
    }

    public void standby() {
        leftMotor.set(ShooterConstants.Control.STANDBY_SPEED);
        rightMotor.set(ShooterConstants.Control.STANDBY_SPEED);
    }

    /**
     * Set Shootertransportation motor to the specified speed.
     * <p>(set the speed in {@link ShooterConstants.Control})
     */

    public void ShooterTransport(){
        transMotor.set(ShooterConstants.Control.SHOOTER_TRANS_SPEED);
    }


    /**
     * Set transportation motor to the specified speed.
     * <p>(set the speed in {@link ShooterConstants.Control})
     */
    public void transport() {
        transMotor.set(ShooterConstants.Control.TRANS_SPEED);
    }

    /**
     * Reverse the transportation motor.
     */
    public void antiTransport() {
        transMotor.set(-ShooterConstants.Control.TRANS_SPEED);
    }

    /**
     * Stop the shooter motors.
     */
    public void stopShoot() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    /**
     * Stop the Shootertransportation motor.
     */
    public void stopShooterTransport(){
        transMotor.stopMotor();
    }

    /**
     * Stop the transportation motor.
     */
    public void stopTransport() {
        transMotor.stopMotor();
    }

    /**
     * Stop all motors.
     */
    public void stopAll() {
        stopShoot();
        stopTransport();
        stopShooterTransport();
        
    }

    /**
     * Set the shooter to the desired position.
     * @param position  Desired shooter position, will be applied using {@link com.revrobotics.SparkPIDController}
     */
    public void setPosition(double position) {
        anglePIDController.setReference(position, ControlType.kPosition);
        desiredAngle = position;
    }

    public void shootTwo() {
        leftMotor.set(0.4);
        rightMotor.set(0.4);
    }

    /**
     * Get the sensor value of the note sensor.
     * @return Whether the digital sensor is detecting a note.
     */
    public boolean noteDetected() {
        return !noteSensor.get();
    }

    public boolean autoTurnReady(){
        boolean minusdeltaDeg = (-limelight.deltaRobotHeadingDeg() - limelight.deltaTargetDeg()) > -3;
        boolean plusdeltaDeg = (-limelight.deltaRobotHeadingDeg() - limelight.deltaTargetDeg()) < 3;
        return (minusdeltaDeg && plusdeltaDeg);
    }

    /**
     * @return true when spinning speed and angle are ready
     */
    public boolean readyToShoot() {
        
        // boolean leftReady = Math.abs(leftMotorEncoder.getVelocity() - desiredSpeed) <= 50;
        // //        boolean rightReady = Math.abs(rightMotorEncoder.getVelocity() - desiredSpeed) <= 50;
        // boolean angleReady = Math.abs(angleEncoder.getPosition() - desiredAngle) <= 2;
        return velocityReady();
    }

    public boolean angleReady() {
        return Math.abs(angleEncoder.getPosition() - desiredAngle) <= 2;
    }

    public boolean velocityReady(){
        // return Math.abs(angleEncoder.getVelocity() - desiredSpeed) <= 1000;
        // return Math.abs(ShooterConstants.Control.SHOOT_VELOCITY - angleEncoder.getVelocity() ) <= 800;
        return Math.abs(ShooterConstants.Control.SHOOT_VELOCITY ) >= 3000;
        // return Math.abs(angleEncoder.getPosition() - desiredAngle) <= 2;
    }

    public double currentPosition() {
        return angleEncoder.getPosition();
    }

    /**
     * set the angle motor to the origin position
     */
    public void originAngle(){
        anglePIDController.setReference(ShooterConstants.Control.ORIGIN_POSITION, ControlType.kPosition);
    }
    public void AmpAngle(){
        anglePIDController.setReference(ShooterConstants.Control.AMP_POSITION, ControlType.kPosition);
    }
    public void NearShootAngle(){
        anglePIDController.setReference(ShooterConstants.Control.NEARSHOOT_POSITION, ControlType.kPosition);
    }
    public void up() {
        angleMotor.set(0.5);
    }

    public void down() {
        angleMotor.set(-0.5);
    }

    public void topAngle(){
        anglePIDController.setReference(ShooterConstants.Control.TOP_POSITION, ControlType.kPosition);
    }

    public void stopAngle() {
        angleMotor.stopMotor();
    }

    public void intakeAngle(){
        anglePIDController.setReference(ShooterConstants.Control.INTAKE_POSITION, ControlType.kPosition);
    }

    public double currentVel(){
        return leftMotorEncoder.getVelocity();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Position", angleEncoder.getPosition());
        SmartDashboard.putNumber("Shooter Velocity", leftMotorEncoder.getVelocity());
        SmartDashboard.putBoolean("NOTE Detected?", noteDetected());
        SmartDashboard.putBoolean("Correct Angle?", angleReady());
        SmartDashboard.putBoolean("Correct Velocity?", velocityReady());
        SmartDashboard.putBoolean("Ready?", readyToShoot());
    }

}
