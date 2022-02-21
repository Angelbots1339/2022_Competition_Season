// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.utils.ShooterProfiles;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * Auto points: <link>https://cad.onshape.com/documents/22cfb86daabd8968995686a8/w/eb0abfda5631fc44d9f905bf/e/ddfc9c84d7e050e02027c18c</link>
 */
public final class Constants {

    public final static String CANIVORE_NAME = "rio";


    public final static class JoystickConstants{

        public final static int MAIN_JOYSTICK = 0;
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
    }
    /**
     * All length units in meters 
     */
    public final static class DriveConstants{
        //general 
        public final static double MAX_DRIVE_OUTPUT_PERCENT = 0.75;
        public final static boolean USE_LIMELIGHT_FIRST = false;
        public final static double ROT_SCALE = 0.65;

        //Motor ports
        public final static int LEFT_MOTOR_TOP_PORT = 2; 
        public final static int LEFT_MOTOR_FRONT_PORT = 1; 
        public final static int LEFT_MOTOR_BACK_PORT = 3;
        public final static int RIGHT_MOTOR_TOP_PORT = 5; 
        public final static int RIGHT_MOTOR_FRONT_PORT = 4; 
        public final static int RIGHT_MOTOR_BACK_PORT = 6; 
        
        
        //Drive base values
        public final static double TRACK_WIDTH_METERS = 0.55128; // Center of left wheel to center of right wheel
        public final static double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public final static double WHEEL_ROT_PER_MOTOR_ROT = 1/6.67;
        public final static double CLICKS_PER_ROT = 2048;
        public final static DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
        public final static boolean RIGHT_INVERTED = true;
        public final static boolean LEFT_INVERTED = false;
        public final static double CLICKS_TO_METERS = 1 / CLICKS_PER_ROT
                * WHEEL_ROT_PER_MOTOR_ROT * WHEEL_DIAMETER_METERS * Math.PI;
        public final static boolean GYRO_INVERTED = true;


        //Slew Rate Limiters
        public final static double DECELERATION_SLEW_RATE_LIMITER = 2; //max speed percent change per second
        public final static double ACCELERATION_SLEW_RATE_LIMITER = 3; //max speed percent change per second

        /* Checking robot kinematics:
        Origin of the robot is the center of rotation when leftVelocity = -rightVelocity
        */

        //PID 
        public final static double LEFT_KP = 3.6052;
        public final static double RIGHT_KP = 3.6052;

        /* Checking kP:
        Graph the input motor values from the desired motor values over time
        */

        //Motion profiling New Values
        public final static double KS = 0.53002; // Volts
        public final static double KV = 2.4483;  // Volts * Seconds / Meters 
        public final static double KA = 0.6174; // Volts * Seconds^2 / Meters

        //Ramsete
        public final static double KB = 2;
        public final static double ZETA = 0.7;
        
        /* Checking kV:
        kV = voltage / free speed (meters per second)
        free speed = free speed of the motor times the wheel circumference divided by the gear reduction
        kV = 12v / (5380rpm / 60s * 0.1016m * pi / 6.67)
        kV (theoretical) ~ 2.797
        */

    }
    public final static class IntakeConstants{

        public final static int INTAKE_MOTOR_PORT = 7; // 7
        public final static int INDEXER_RIGHT_PORT = 9; // 9
        public final static int INDEXER_LEFT_PORT = 8; // 8
        public final static double MAX_INDEXER_PERCENT = 0.5;
        public final static double MAX_INTAKE_PERCENT = 1;        

        public final static boolean INDEXER_LEFT_INVERTED = true;
        public final static boolean INDEXER_RIGHT_INVERTED = false;
        
        public final static boolean INTAKE_INVERSE = false;

        public final static int COLOR_SENSOR_PROXIMITY_THRESHOLD = 200; // 0 (closest) to 2047 (furthest)
    }
    public static final class LoaderConstants{
        public final static int LOADER_PORT = 10; // 10
        public final static boolean LOADER_INVERSE = true;
        public final static double MAX_LOADER_SPEED = 0.6;

    }

    public static final class ClimberConstants{
        public static final int ROTATOR_LEFT_PORT = 16;  // 16 
        public static final int ROTATOR_RIGHT_PORT = 17; // 17
        public static final int EXTENDER_LEFT_PORT = 14; // 14
        public static final int EXTENDER_RIGHT_PORT = 15; // 15
        public static final boolean ROTATOR_LEFT_INVERTED = true;
        public static final boolean ROTATOR_RIGHT_INVERTED = false;
        public static final boolean EXTENDER_LEFT_INVERTED = false;
        public static final boolean EXTENDER_RIGHT_INVERTED = true;
      
        public static final int ROTATOR_LEFT_LIMIT_PORT = 1;
        public static final int ROTATOR_RIGHT_LIMIT_PORT = 0;
        public static final double LIMIT_SWITCH_DEBOUNCE_SECONDS = 0.02;

        public static final double EXTENDER_TOP_LIMIT = 0.70;
        public static final double EXTENDER_BOTTOM_LIMIT = -0.04;

        public final static double ROTATOR_BACK_LIMIT_DEG = 0;
        public final static double ROTATOR_FRONT_LIMIT_DEG = 28.5;

        public final static double MAX_ROTATOR_VOLTS = 2;
        public final static double MAX_EXTENDER_VOLTS = 5;
        public final static double MAX_EXTENDER_VOLTS_RETRACT = 7;
        public final static double EXTENDER_SETPOINT_THRESHOLD = .01; // m
        public final static double ROTATION_SETPOINT_THRESHOLD = .5; // deg

       
        private static final double MOTOR_ROT_PER_SPOOL_ROT = 16/1;
        private static final double MOTOR_ROT_PER_ARM_ROT = 30.0/4100.0;
        private static final double SPOOL_CIRCUM = Units.inchesToMeters(1.25) * Math.PI;
        public static final double LENGTH_PER_CLICK = (1 / DriveConstants.CLICKS_PER_ROT) * SPOOL_CIRCUM / MOTOR_ROT_PER_SPOOL_ROT;

        public static final double AUTO_EXTENSION_SETPOINT = 0.7351;
        public static final double AUTO_ROTATION_SETPOINT = 21.76;
        public static final double AUTO_ROTATION_BACK_SETPOINT = 15;
        
        public final static double GET_DEGREES_FROM_CLICKS(double Clicks){
            return Clicks / DriveConstants.CLICKS_PER_ROT * MOTOR_ROT_PER_ARM_ROT * 360;
        }
    }

    public final static class ShooterConstants {
        public final static int LEFT_POWER_WHEEL = 11;  // 11
        public final static int RIGHT_POWER_WHEEL = 12; // 12 
        public final static int AIM_WHEEL = 13;  // 13

        public final static double POWER_WHEEL_KF = 0.0;
        public final static double POWER_WHEEL_KP = 0.25998;
        public final static double POWER_WHEEL_KI = 0;
        public final static double POWER_WHEEL_KD = 0;

        public final static double AIM_WHEEL_KF = 0.0;
        public final static double AIM_WHEEL_KP = 0.001;
        public final static double AIM_WHEEL_KI = 0;
        public final static double AIM_WHEEL_KD = 0;

        public final static double AIM_WHEEL_TOLERANCE = 50;
        public final static double POWER_WHEEL_TOLERANCE = 50;
        
        public final static boolean LEFT_POWER_WHEEL_INVERTED = true;
        public final static boolean RIGHT_POWER_WHEEL_INVERTED = false;

        public final static ShooterProfiles SHOOTER_PROFILE_HIGH = new ShooterProfiles(() -> 2260, () -> 2900, () -> .43, () -> .53);
        public final static ShooterProfiles SHOOTER_PROFILE_LOW = new ShooterProfiles(() -> 1300, () -> 1520, () -> .28, () -> .3);
    }

    public final static class MultiplexerConstants {

        public static final byte DEFAULT_ADDRESS = 0x70;
        public static final Port DEFAULT_PORT = Port.kMXP;
    }
    public final static class LimelightConstants{
        public static enum entryType{
            VALID_TARGETS, HORIZONTAL_OFFSET, VERTICAL_OFFSET, TARGET_AREA, SKEW, LATENCY, SHORTEST_SIDE, LONGEST_SIDE, HORIZONTAL_BOUNDS, VERTICAL_BOUNDS, ACTIVE_PIPELINE, POSE_3D, LED_MODE, CAM_MODE, PIPELINE, STREAM, SNAPSHOT;

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

