package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private final CANSparkMax pipeIntake = new CANSparkMax(IntakeConstants.Config.PIPE_ID, MotorType.kBrushless);
    private final CANSparkMax angleIntake = new CANSparkMax(IntakeConstants.Config.ANGLE_ID, MotorType.kBrushless);
    private final CANSparkMax conveyor = new CANSparkMax(IntakeConstants.Config.CONVEYOR_ID, MotorType.kBrushless);
    private final AbsoluteEncoder intakeEncoder;
    private final SparkPIDController intakePIDController;

    /**
     * set PID using SparkPIDController
     * @param sparkPIDController sparkPIDController which you want to set PID for
     * @param p set P
     * @param i set I
     * @param d set D
     */
    private void setPID(SparkPIDController sparkPIDController, double p, double i, double d){
        sparkPIDController.setP(p);
        sparkPIDController.setI(i);
        sparkPIDController.setD(d);
    }

    public Intake(){
        pipeIntake.restoreFactoryDefaults();
        angleIntake.restoreFactoryDefaults();
        conveyor.restoreFactoryDefaults();

        pipeIntake.setIdleMode(IdleMode.kBrake);
        angleIntake.setIdleMode(IdleMode.kBrake);
        conveyor.setIdleMode(IdleMode.kBrake);

        pipeIntake.setInverted(IntakeConstants.Config.PIPE_INVERTED);
        angleIntake.setInverted(IntakeConstants.Config.ANGLE_INVERTED);
        conveyor.setInverted(IntakeConstants.Config.CONVEYOR_INVERTED);

        pipeIntake.setSmartCurrentLimit(IntakeConstants.Config.CURRENT_LIMIT);
        angleIntake.setSmartCurrentLimit(IntakeConstants.Config.CURRENT_LIMIT);
        conveyor.setSmartCurrentLimit(IntakeConstants.Config.CURRENT_LIMIT);

        intakeEncoder = angleIntake.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        intakeEncoder.setInverted(true);
        intakeEncoder.setPositionConversionFactor(360);  // 0~1 -> 0~360

        intakePIDController = angleIntake.getPIDController();
        intakePIDController.setFeedbackDevice(intakeEncoder);
        setPID(intakePIDController, IntakeConstants.AnglePIDF.P, IntakeConstants.AnglePIDF.I, IntakeConstants.AnglePIDF.D);
        // configuration for Smart Motion
        intakePIDController.setSmartMotionMaxAccel(2000, 0);
        intakePIDController.setSmartMotionMaxVelocity(2000, 0);
        /* enable PID wrapping
         * after wrapping:
         *   set "0 -> 350", motor will go -10 instead of +350
         */
        intakePIDController.setPositionPIDWrappingEnabled(true);
        intakePIDController.setPositionPIDWrappingMinInput(0);
        intakePIDController.setPositionPIDWrappingMaxInput(360);
        intakePIDController.setOutputRange(-0.3, 1);

        pipeIntake.burnFlash();
        angleIntake.burnFlash();
        conveyor.burnFlash();

        backToZero();
    }

    public void shoot(){
        pipeIntake.set(IntakeConstants.Control.SHOOT_SPEED);
    }

    public void suck(){
        pipeIntake.set(IntakeConstants.Control.SUCK_SPEED);
    }

    public void slowSuck() {
        pipeIntake.set(0.8);
    }

    public void startConvey() {
        conveyor.set(IntakeConstants.Control.CONVEYOR_SPEED);
    }

    public void safeIntake(){
        conveyor.set(0.2);
    }

    public void reverseConvey() {
        conveyor.set(-0.8);
    }

    public void stopIntake() {
        pipeIntake.stopMotor();
    }

    public void stopConvey() {
        conveyor.stopMotor();
    }

    public void stopAll() {
        stopIntake();
        stopConvey();
    }

    /**
     * set intake angle motor to the floor position
     * <p> (set floor angle at {@link IntakeConstants.Control})
     */
    public void setFloorAngle(){
        intakePIDController.setReference(IntakeConstants.Control.FLOOR_POSITION, ControlType.kPosition);
    }

    public void setReverseAngle(){
        intakePIDController.setReference(IntakeConstants.Control.REVERSE_POSITION, ControlType.kPosition);
    }
    /**
     * set intake angle motor to the amp position
     * <p> (set amp angle at {@link IntakeConstants.Control})
     */
    public void setAmpAngle(){
        intakePIDController.setReference(IntakeConstants.Control.AMP_POSITION, ControlType.kPosition);
    }

    /**
     * set intake angle motor to the source position
     * <p> (set trap angle at {@link IntakeConstants.Control})
     */
    public void setSourceAngle(){
        intakePIDController.setReference(IntakeConstants.Control.SOURCE_POSITION, ControlType.kPosition);
    }

    /**
     * set intake angle motor to the origin position
     * <p> (set origin angle at {@link IntakeConstants.Control})
     */
    public void backToZero(){
        // intakePIDController.setReference(8, ControlType.kPosition);
        intakePIDController.setReference(IntakeConstants.Control.ORIGIN_POSITION, ControlType.kPosition);
    }

    public void up() {
        angleIntake.set(0.5);
    }

    public void down() {
        angleIntake.set(-0.5);
    }

    public void stopAngle() {
        angleIntake.stopMotor();
    }

    public void conveyorShoot() {
        conveyor.set(IntakeConstants.Control.CONVEYOR_SPEED);
    }

    public double intakeVel(){
        return pipeIntake.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", intakeEncoder.getPosition());
        SmartDashboard.putNumber("Intake Velocity", pipeIntake.getEncoder().getVelocity());
    }
}

