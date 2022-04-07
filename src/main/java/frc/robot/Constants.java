// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.utils.ColorRange;
import frc.robot.utils.ShooterProfiles;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 * 
 * Auto points:
 * <link>https://cad.onshape.com/documents/22cfb86daabd8968995686a8/w/eb0abfda5631fc44d9f905bf/e/ddfc9c84d7e050e02027c18c</link>
 */
public final class Constants {

    public final static String CANIVORE_NAME = "rio";

    public static boolean isMotorStalling(WPI_TalonFX motor) {
        double voltage = motor.getMotorOutputVoltage();
        return motor.getStatorCurrent() >= 1.4 * voltage * voltage + 4.26 * voltage;
    }

    public final static class JoystickConstants {
        // USB Controller Ports
        public final static int MAIN_JOYSTICK = 0;

        // Button mappings
        public final static int BUTTON_A = 1;
        public final static int BUTTON_B = 2;
        public final static int BUTTON_X = 3;
        public final static int BUTTON_Y = 4;
        public final static int LEFT_BUMPER = 5;
        public final static int RIGHT_BUMPER = 6;
        public final static int LEFT_MENU_BUTTON = 7;
        public final static int RIGHT_MENU_BUTTON = 8;
        public final static int LEFT_JOYSTICK_BUTTON = 9;
        public final static int RIGHT_JOYSTICK_BUTTON = 10;

        // Timings
        public final static double TELEOP_TIME = 2 * 60 + 15;
        public final static double CLIMB_TIME = 30;
    }

    /**
     * All length units in meters
     */
    public final static class DriveConstants {
        // General
        public final static double MAX_DRIVE_OUTPUT_PERCENT = 0.75;
        public final static boolean USE_LIMELIGHT_FIRST = false;
        public final static double ROT_SCALE = 0.65;

        // Slew Rate Limiters
        public final static double DECELERATION_SLEW_RATE_LIMITER = 2; //2  // max speed percent change per second
        public final static double ACCELERATION_SLEW_RATE_LIMITER = 3; //3  // max speed percent change per second

        // Ports
        public final static int LEFT_MOTOR_TOP_PORT = 2;
        public final static int LEFT_MOTOR_FRONT_PORT = 1;
        public final static int LEFT_MOTOR_BACK_PORT = 3;
        public final static int RIGHT_MOTOR_TOP_PORT = 5;
        public final static int RIGHT_MOTOR_FRONT_PORT = 4;
        public final static int RIGHT_MOTOR_BACK_PORT = 6;

        // Config
        public final static double CENTER_DIST_BACK_BUMPER = 31.85541;
        public final static double TRACK_WIDTH_METERS = 0.5470525; // Center of left wheel to center of right wheel
        public final static double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.18);
        public final static double WHEEL_ROT_PER_MOTOR_ROT = 1 / 6.67;
        public final static double CLICKS_PER_ROT = 2048;
        public final static DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);
        public final static boolean RIGHT_INVERTED = true;
        public final static boolean LEFT_INVERTED = false;
        public final static double CLICKS_TO_METERS = 1 / CLICKS_PER_ROT
                * WHEEL_ROT_PER_MOTOR_ROT * WHEEL_DIAMETER_METERS * Math.PI;
        public final static boolean GYRO_INVERTED = true;

        // PID / Motion Profiling
        public final static double LEFT_KP = 3.5;
        public final static double RIGHT_KP = 3.5; // 2.8639
        public final static double KS = 0.73816; // Volts
        public final static double KV = 2.1836; // Volts * Seconds / Meters
        public final static double KA = 0.25419; // Volts * Seconds^2 / Meters
        public final static double KB = 3.8; // 3.5
        public final static double ZETA = 0.4; // 0.4

        /*
         * Checking kP:
         * Graph the input motor values from the desired motor values over time
         */

        /*
         * Checking kV:
         * kV = voltage / free speed (meters per second)
         * free speed = free speed of the motor times the wheel circumference divided by
         * the gear reduction
         * kV = 12v / (6380rpm / 60s * 0.1016m * pi / 6.67)
         * kV (theoretical) ~ 2.36
         */
    }

    public final static class IntakeConstants {

        // General
        public final static double MAX_INDEXER_PERCENT = 0.5;
        public final static double MAX_INTAKE_PERCENT = 1;
        public final static double INTAKE_DEPLOY_SPEED = .2;
        public final static double INTAKE_DEPLOY_TIME = .4;
        public final static double INTAKE_RETRACT_MAX_VOLTS = .4;

        public final static double RETRACTION_THRESHOLD = 0.2;
        public final static double DEPLOY_SETPOINT = 1;
        public final static double RETRACTION_SETPOINT = 0;
    


        // Ports
        public final static int INTAKE_MOTOR_PORT = 7;
        public final static int INDEXER_RIGHT_PORT = 9;
        public final static int INDEXER_LEFT_PORT = 8;

        public final static int INTAKE_RETRACT_LEFT_PORT = 18;
        public final static int INTAKE_RETRACT_RIGHT_PORT = 19;

        // Config
        public final static boolean INDEXER_LEFT_INVERTED = true;
        public final static boolean INDEXER_RIGHT_INVERTED = false;
        public final static boolean INTAKE_INVERTED = false;
        public final static int COLOR_SENSOR_PROXIMITY_THRESHOLD = 100; // 0 (closest) to 2047 (furthest)
    }

    public static final class LoaderConstants {

        // General
        public final static double MAX_LOADER_SPEED = 0.8;
        public final static double MAX_LOADER_INTAKE_SPEED = 0.3;
        public final static double REJECT_TIME = .5; //s
        public final static double REVERSE_TIME = .3;

        // Ports
        public final static int LOADER_PORT = 10;

        // Config
        public final static boolean LOADER_INVERSE = true;

        public final static ColorRange RED = new ColorRange(990, 500, 100, 600);
        public final static ColorRange BLUE = new ColorRange(280, 620, 540, 600);    
    }

    public static final class ClimberConstants {

        // General
        public final static double MAX_ROTATOR_VOLTS = 2;
        public final static double MAX_EXTENDER_VOLTS = 9;
        public final static double PULLUP_VOLTS = 7;
        public final static double MANUAL_UP_VOLTS = 8;
        public final static double DROP_EXTENDER_VOLTS = 4;
        public final static double EXTENDER_SETPOINT_THRESHOLD = .02; // m
        public final static double ROTATION_SETPOINT_THRESHOLD = 2; // deg

        // Ports
        public static final int ROTATOR_LEFT_PORT = 16;
        public static final int ROTATOR_RIGHT_PORT = 17;
        public static final int EXTENDER_LEFT_PORT = 14;
        public static final int EXTENDER_RIGHT_PORT = 15;
        public static final int ROTATOR_LEFT_FRONT_LIMIT_PORT = 1;
        public static final int ROTATOR_RIGHT_FRONT_LIMIT_PORT = 0;
        public static final int ROTATOR_RIGHT_BACK_LIMIT_PORT = 2;
        public static final int ROTATOR_LEFT_BACK_LIMIT_PORT = 3;
        public final static int LEFT_ENCODER_PORT = 5;
        public final static int RIGHT_ENCODER_PORT = 4;

        // Soft stops
        public static final double EXTENDER_TOP_LIMIT = 0.75;
        public static final double EXTENDER_BOTTOM_LIMIT = -0.04;
        public final static double ROTATOR_BACK_LIMIT_DEG = 0;
        public final static double ROTATOR_FRONT_LIMIT_DEG = 25.5;

        // Config
        public static final boolean ROTATOR_LEFT_INVERTED = true;
        public static final boolean ROTATOR_RIGHT_INVERTED = false;
        public static final boolean EXTENDER_LEFT_INVERTED = false;
        public static final boolean EXTENDER_RIGHT_INVERTED = true;
        public static final double LIMIT_SWITCH_DEBOUNCE_SECONDS = 0.02;
        private static final double MOTOR_ROT_PER_SPOOL_ROT = 16 / 1;
        private static final double MOTOR_ROT_PER_ARM_ROT = 30.0 / 4100.0;
        private static final double SPOOL_CIRCUM = Units.inchesToMeters(2) * Math.PI;
        public static final double LENGTH_PER_CLICK = (1 / DriveConstants.CLICKS_PER_ROT) * SPOOL_CIRCUM
                / MOTOR_ROT_PER_SPOOL_ROT;

        public final static double GET_DEGREES_FROM_CLICKS(double Clicks) {
            return Clicks / DriveConstants.CLICKS_PER_ROT * MOTOR_ROT_PER_ARM_ROT * 360;
        }
    }

    public final static class ShooterConstants {

        // General
        public final static double AIM_WHEEL_TOLERANCE = 70; // rpm
        public final static double POWER_WHEEL_TOLERANCE = 70; // rpm
        public final static ShooterProfiles SHOOTER_PROFILE_HIGH = new ShooterProfiles(() -> 1260, () -> 3015); // () -> 1400, () -> 3350
        public final static ShooterProfiles SHOOTER_PROFILE_LOW = new ShooterProfiles(() -> 1200, () -> 1100);
        public final static ShooterProfiles SHOOTER_PROFILE_REJECT = new ShooterProfiles(() -> 700, () -> 300);

        // Ports
        public final static int LEFT_POWER_WHEEL = 11; // 11
        public final static int RIGHT_POWER_WHEEL = 12; // 12
        public final static int AIM_WHEEL = 13; // 13

        // PID / Motion Profiling
        public final static double POWER_WHEEL_KF = 0.00172528;
        public final static double POWER_WHEEL_KB = 0.823253;
        public final static double POWER_WHEEL_KP = 0.001;
        public final static double POWER_WHEEL_KI = 0;
        public final static double POWER_WHEEL_KD = 0;
        public final static double AIM_WHEEL_KF = 0.00181965;
        public final static double AIM_WHEEL_KB = 0.5523;
        public final static double AIM_WHEEL_KP = 0.0008;
        public final static double AIM_WHEEL_KI = 0;
        public final static double AIM_WHEEL_KD = 0;

        // Config
        public final static boolean LEFT_POWER_WHEEL_INVERTED = true;
        public final static boolean RIGHT_POWER_WHEEL_INVERTED = false;
    }

    public final static class MultiplexerConstants {

        // Ports
        public static final byte DEFAULT_ADDRESS = 0x70;
        public static final Port DEFAULT_PORT = Port.kMXP;
    }

    public final static class AutoConstants {

        // General
        public static final double SHOOT_TIME_2B = 1.55; // s
        public static final double SHOOT_TIME_1B = 1; // s
        public static final double TURN_VOLTS = 3;
        public static final double HALF_TURN_TIME = .75;

    }

    public final static class TurnConstants {

       public final static double TURN_THRESHOLD = 0; 
    }

    public final static class LimelightConstants {
        public static enum entryType {
            VALID_TARGETS, HORIZONTAL_OFFSET, VERTICAL_OFFSET, TARGET_AREA, SKEW, LATENCY, SHORTEST_SIDE, LONGEST_SIDE,
            HORIZONTAL_BOUNDS, VERTICAL_BOUNDS, ACTIVE_PIPELINE, POSE_3D, LED_MODE, CAM_MODE, PIPELINE, STREAM,
            SNAPSHOT;

            @Override
            public String toString() {
                switch (this) {
                    case VALID_TARGETS:
                        return "tv";
                    case HORIZONTAL_OFFSET:
                        return "tx";
                    case VERTICAL_OFFSET:
                        return "ty";
                    case TARGET_AREA:
                        return "ta";
                    case SKEW:
                        return "ts";
                    case LATENCY:
                        return "tl";
                    case SHORTEST_SIDE:
                        return "tshort";
                    case LONGEST_SIDE:
                        return "tlong";
                    case HORIZONTAL_BOUNDS:
                        return "thor";
                    case VERTICAL_BOUNDS:
                        return "tvert";
                    case ACTIVE_PIPELINE:
                        return "getpipe";
                    case POSE_3D:
                        return "camtran";
                    case LED_MODE:
                        return "ledMode";
                    case CAM_MODE:
                        return "camMode";
                    case PIPELINE:
                        return "pipeline";
                    case STREAM:
                        return "stream";
                    case SNAPSHOT:
                        return "snapshot";

                    default:
                        return "";
                }
            }
        }
    }
}
