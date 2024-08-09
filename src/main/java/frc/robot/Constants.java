package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import swervelib.math.Matter;

/*
 * CAN ID rules:
 *  0-9: Drive Base (configured in json)
 *  10-19: Shooter
 *  20-29: Intake
 *  30-39: Climber
 *  40+: Other
 */
public class Constants {
    public static class Chassis {
        public static final double ROBOT_MASS = 40;  // kg
        public static final double LOOP_TIME = 0.13;  // s, 20ms + 110ms SPARK MAX velocity lag
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double MODULE_MAX_SPEED = 5.88264;  // L3, NEO
    }

    public static class AutoDrivePIDF {
        public static final double P = 4.5;
        public static final double I = 0.00;
        public static final double D = 0;
        public static final double I_ZONE = 0;
    }

    public static class AutoTurnPIDF {
        public static final double P = 1.3;
        public static final double I = 0.0;
        public static final double D = 0.00;
        public static final double I_ZONE = 0.0;
    }

    public static class AutoAimPID {
        public static final double P = 0.06;
        public static final double I = 0;
        public static final double D = 0;
    }

    
    public static class ShooterConstants {
        public static class Config {
            // CAN ID: 10-19
            public static final int L_ID = 11;
            public static final int R_ID = 12;
            public static final int TRANS_ID = 13;
            public static final int ANGLE_ID = 10;
            public static final boolean SHOOTER_INVERTED = true;
            public static final boolean TRANS_INVERTED = true;
            public static final boolean ANGLE_INVERTED = true;
            public static final int SENSOR_ID = 9;
            public static final int CURRENT_LIMIT = 35;
            public static final int ANGLE_OFFSET = 9;
        }

        public static class Control {
            public static final double 
            SHOOT_VELOCITY = 3200;  // Absolute
            public static final double AMP_VELOCITY = 1650;  
            public static final double SHOOT_NEAR_VELOCITY = 3970;
            public static final double SUCK_SPEED = -0.3;  // Relative
            public static final double STANDBY_SPEED = 0.2;
            public static final double ORIGIN_POSITION = 5;
            public static final double AMP_POSITION = 15;
            // public static final double INTAKE_POSITION = 29;  //9.5
            public static final double INTAKE_POSITION = 16;  //9.5
            public static final double REVERSE_POSITION = 20;
            public static final double SOURCE_POSITION = 28.8;
            public static final double TOP_POSITION = 46.5;
            public static final double 
            NEARSHOOT_POSITION = 43;
            public static final double TRANS_SPEED = 0.1;
            // public static final double SHOOTER_TRANS_SPEED = 0.5;
            public static final double SHOOTER_TRANS_SPEED = 0.8;
        }

        public static class AnglePIDF {
            public static final double P = 0.01;
            public static final double I = 0.000001;
            public static final double D = 0.0;
        }

        public static class LeftPIDF {
            public static final double P = 0.001;
            public static final double I = 0;
            public static final double D = 0;
        }

        public static class RightPIDF {
            public static final double P = 0;
            public static final double I = 0;
            public static final double D = 0;
        }
    }

    public static class IntakeConstants {
        public static class Config {
            // CAN ID: 20-29
            public static final int PIPE_ID = 22;
            public static final int ANGLE_ID = 20;
            public static final int CONVEYOR_ID = 21;
            public static final boolean PIPE_INVERTED = false;
            public static final boolean ANGLE_INVERTED = false;
            public static final boolean CONVEYOR_INVERTED = false;
            public static final int CURRENT_LIMIT = 35;
        }

        public static class AnglePIDF {
            public static final double P = 0.004050;
            public static final double I = 0.000001;
            public static final double D = 0;
        }

        public static class Control {
            // public static final double FLOOR_POSITION = 129.5;
            public static final double FLOOR_POSITION = 234;
            public static final double REVERSE_POSITION = 235;
            public static final double AMP_POSITION = 350;
            public static final double SOURCE_POSITION = 10;
            public static final double ORIGIN_POSITION = 0;
            public static final double SHOOT_SPEED = -0.8;
            public static final double SUCK_SPEED = 0.6;
            public static final double CONVEYOR_SPEED = 0.4;
        }
    }

    public static class ClimberConstants {
        public static class Config {
            // CAN ID: 30-39
            public static final int F_ID = 30;
            public static final int R_ID = 31;
            public static final boolean INVERTED = false;
            public static final double MAX_ACCEL = 500;
            public static final double MAX_VELOCITY = 200;
        }

        public static class Control {
            public static final double FLOOR = 0;
            public static final double AMP = 20;
            public static final double BALANCE = 30;
        }

        public static class PIDF {
            public static final double P = 0.1;
            public static final double I = 0;
            public static final double D = 0;
            public static final double F = 0;
        }
    }

    public static class OthersConstants {
        public static final int CANDLE_ID = 40;
    }

    public static class VisionConstants {
        public static final String NAME = "limelight";
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0.46, 0.365, -0.7,  // Unit: Meters
                new Rotation3d(0, 0.125 * Math.PI, 0));  // Unit: Radians
    }
}
