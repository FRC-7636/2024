package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OthersConstants;

public class Candle extends SubsystemBase {
    private final CANdle candle = new CANdle(OthersConstants.CANDLE_ID, "cantivore");
    private final Limelight limelight = new Limelight();

    //private final LarsonAnimation normalState = new LarsonAnimation(255, 255, 255);
    private final StrobeAnimation detectedState = new StrobeAnimation(255, 0, 255);
    private final StrobeAnimation shootingState = new StrobeAnimation(0, 255, 0);
    private final StrobeAnimation intakingState = new StrobeAnimation(0, 0, 255);
    private final StrobeAnimation ampState = new StrobeAnimation(255, 0, 0);
    private final RainbowAnimation normalState = new RainbowAnimation();
    private final double blinkSpeed = 0.4;

    public Candle() {
        candle.configFactoryDefault();

        candle.clearAnimation(0);
        candle.setLEDs(255, 255, 255);
    }

    public void backToNormalState() {
        candle.animate(normalState);
        normalState.setSpeed(0.75);
        normalState.setNumLed(60);
       // normalState.setSize(5);
    }

    public void noteDetectedState() {
        candle.animate(detectedState);
        detectedState.setSpeed(blinkSpeed);
    }

    public void Shooting(){
        candle.animate(shootingState);
        shootingState.setSpeed(blinkSpeed);
    }

    public void Intaking(){
        candle.animate(intakingState);
        intakingState.setSpeed(blinkSpeed);
    }

    public void AMPing(){
        candle.animate(ampState);
        ampState.setSpeed(blinkSpeed);
    }
    
    @Override
    public void periodic() {
        double shooterVelocity = SmartDashboard.getNumber("Shooter Velocity", 0);
        double intakeVelocity = SmartDashboard.getNumber("Intake Velocity", 0);
        boolean noteDetect = SmartDashboard.getBoolean("NOTE Detected?", false);
        double climberPos = SmartDashboard.getNumber("Climber Position R", 0);
        double intakePos = SmartDashboard.getNumber("Intake Position", 0);
        boolean ready = SmartDashboard.getBoolean("Ready?", false);
        if (ready & noteDetect & (climberPos < 10)){
            Shooting();
            limelight.LEDoff();
        }
        else if ((intakeVelocity > 1) & !noteDetect & (climberPos < 10) & (intakePos > 100)){
            Intaking();
            limelight.blink();;
        }
        else if ((climberPos > 20) & !noteDetect){
            AMPing();
            limelight.LEDoff();
        }
        else if (noteDetect & (climberPos < 10)) {
            noteDetectedState();
            limelight.LEDoff();
        }
        else if(!noteDetect & (climberPos < 10)){
            backToNormalState();
            limelight.LEDoff();
        }
    }
}

