package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final TalonFX frontMotor = new TalonFX(ClimberConstants.Config.F_ID, "cantivore");
    private final TalonFX rearMotor = new TalonFX(ClimberConstants.Config.R_ID, "cantivore");

    public Climber() {
        var frontMotorConfigurator = frontMotor.getConfigurator();
        var rearMotorConfigurator = rearMotor.getConfigurator();

        // reset the motor to factory defaults
        frontMotorConfigurator.apply(new TalonFXConfiguration());
        rearMotorConfigurator.apply(new TalonFXConfiguration());

        // set the motors to brake mode & invert motors
        frontMotor.setNeutralMode(NeutralModeValue.Brake);
        frontMotor.setInverted(ClimberConstants.Config.INVERTED);
        rearMotor.setNeutralMode(NeutralModeValue.Brake);
        rearMotor.setInverted(ClimberConstants.Config.INVERTED);

        // set feedback sensor as integrated sensor
        frontMotorConfigurator.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        rearMotorConfigurator.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // set maximum acceleration and velocity
        frontMotorConfigurator.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ClimberConstants.Config.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ClimberConstants.Config.MAX_VELOCITY));
        rearMotorConfigurator.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ClimberConstants.Config.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ClimberConstants.Config.MAX_VELOCITY));

        frontMotorConfigurator.setPosition(0);
        rearMotorConfigurator.setPosition(0);

        // setup PIDConfig
        Slot0Configs PIDConfig = new Slot0Configs();
        PIDConfig.kP = ClimberConstants.PIDF.P;
        PIDConfig.kI = ClimberConstants.PIDF.I;
        PIDConfig.kD = ClimberConstants.PIDF.D;
        PIDConfig.kV = ClimberConstants.PIDF.F;  // kF is actually kV...? Better keep it at zero...
        frontMotorConfigurator.apply(PIDConfig);
        rearMotorConfigurator.apply(PIDConfig);
    }

    public void setFloorLevel() {
        frontMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.FLOOR));
        rearMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.FLOOR));
    }

    public void setAmpLevel() {
        frontMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.AMP));
        rearMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.AMP));
    }

    public void setBalanceLevel() {
        frontMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.BALANCE));
        rearMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.BALANCE));
    }

    public void setPosition(double position) {
        frontMotor.setControl(new MotionMagicDutyCycle(position));
        rearMotor.setControl(new MotionMagicDutyCycle(position));
    }

    public void up() {
        frontMotor.set(0.2);
        rearMotor.set(0.2);
    }

    public void down() {
        frontMotor.set(-0.2);
        rearMotor.set(-0.2);
    }

    public void stop() {
        frontMotor.stopMotor();
        rearMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position F", frontMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber Position R", rearMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber Velocity", frontMotor.getVelocity().getValueAsDouble());
    }
}

