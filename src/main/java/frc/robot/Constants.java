// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Clamps and scales input
     * @param in
     * @param limit Hard limit (0, inf)
     * @param factor Factor to multiply by result of clamping by
     * @return
     */
    public final static double limitToFactor(double in, double limit, double factor) {
        return factor * MathUtil.clamp(in, -limit, limit);
    }



    public final static class JoystickConstants{

         public final static int mainJoystick = 0;
         //public final static int secondaryJoystick = 0; //This is optional


    

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

        public final static double JOYSTICK_THRESHOLD = .1;
        
        

    }
    /**
     * All length units in meters 
     */
    public final static class DriveConstants{
        //general 
        public final static double MAX_DRIVE_OUTPUT_PERCENT = 0.25;
        public final static boolean LOG_DATA = true;

        //Motor ports
        public final static int LEFT_MOTOR_TOP_PORT = 2; 
        public final static int LEFT_MOTOR_FRONT_PORT = 1; 
        public final static int LEFT_MOTOR_BACK_PORT = 3;
        public final static int RIGHT_MOTOR_TOP_PORT = 5; 
        public final static int RIGHT_MOTOR_FRONT_PORT = 4; 
        public final static int RIGHT_MOTOR_BACK_PORT = 6; 
        
        //Drive base values
        public final static double TRACK_WIDTH_METERS = Units.inchesToMeters(21.5); // Center of left wheel to center of right wheel
        public final static double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.875);
        public final static double WHEEL_ROT_PER_MOTOR_ROT = 1/6.67;
        public final static double CLICKS_PER_ROT = 2048;
        public final static DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
        public final static boolean RIGHT_INVERTED = true;
        public final static boolean LEFT_INVERTED = false;
        public final static double CLICKS_TO_METERS = 1 / CLICKS_PER_ROT
                * WHEEL_ROT_PER_MOTOR_ROT * WHEEL_DIAMETER_METERS * Math.PI;
        public final static boolean GYRO_INVERTED = true;

        /* Checking robot kinematics:
        Origin of the robot is the center of rotation when leftVelocity = -rightVelocity
        */

        //PID 
        public final static double LEFT_KP = 2.1258;
        public final static double RIGHT_KP = 2.1258;

        /* Checking kP:
        Graph the input motor values from the desired motor values over time
        */

        //Motion profiling
        public final static double KS = 0.5221; // Volts
        public final static double KV = 2.1103;  // Volts * Seconds / Meters 
        public final static double KA = 0.11835; // Volts * Seconds^2 / Meters

        

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

        // TODO Make the ports correct
        public final static int INTAKE_MOTOR_PORT = 7; // 7
        public final static int INDEXER_RIGHT_PORT = 9; // 9
        public final static int INDEXER_LEFT_PORT = 8; // 8
        public final static double MAX_INDEXER_PERCENT = 0.5;
        public final static double MAX_INTAKE_PERCENT = 0.6;

        public final static double MAX_INTAKE_CURRENT = 0;
        

        public final static boolean INDEXER_LEFT_INVERTED = true;
        public final static boolean INDEXER_RIGHT_INVERTED = false;
        
        public final static boolean INTAKE_INVERSE = false;
        
        public final static int SERVO_RIGHT_PORT = 0;
        public final static int SERVO_LEFT_PORT = 1;

        public final static int COLOR_SENSOR_PROXIMITY_THRESHOLD = 200; // 0 to 2047

        
        // TODO Fix color sensor ports
        public final static int COLOR_SENSOR_HIGH_PORT = -1;
        public final static int COLOR_SENSOR_LOW_PORT = -1;

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

        public static final double EXTENDER_TOP_LIMIT = 0.8;
        public static final double EXTENDER_BOTTOM_LIMIT = 0;
        public final static double ROTATOR_BACK_LIMIT_DEG = 0;
        public final static double ROTATOR_FRONT_LIMIT_DEG = 20;


        public final static double MAX_ROTATOR_VOLTS = 3;
        public final static double MAX_EXTENDER_VOLTS = 3;
        public final static double MAX_ROTATOR_SPEED = .5; // m/s
        public final static double MAX_EXTENDER_SPEED = .5; // m/s
        public final static double EXTENDER_SETPOINT_THRESHOLD = .01; // m

        public static final boolean LEFT_IS_LEADER = true;
       
        private static final double MOTOR_ROT_PER_SPOOL_ROT = 16/1;
        private static final double MOTOR_ROT_PER_ARM_ROT = 30.0/4100.0;
        private static final double SPOOL_CIRCUM = Units.inchesToMeters(1.25) * Math.PI;
        public static final double LENGTH_PER_CLICK = (1 / DriveConstants.CLICKS_PER_ROT) * SPOOL_CIRCUM / MOTOR_ROT_PER_SPOOL_ROT;
        
        
        public final static double GET_DEGREES_FROM_CLICKS(double Clicks){
            return Clicks / DriveConstants.CLICKS_PER_ROT * MOTOR_ROT_PER_ARM_ROT * 360;
            //return Math.IEEEremainder(Clicks / DriveConstants.CLICKS_PER_ROT / MOTOR_ROT_PER_ARM_ROT * 360, 360);

        }

        public final static double EXTENDER_FOLLOWER_KP = .05;
        public final static double EXTENDER_FOLLOWER_KI = 0;
        public final static double EXTENDER_FOLLOWER_KD = 0;
        public final static double ROTATOR_FOLLOWER_KP = .05;
        public final static double ROTATOR_FOLLOWER_KI = 0;
        public final static double ROTATOR_FOLLOWER_KD = 0;

        public final static double EXTENDER_LEFT_KS = .05; // Volts
        public final static double EXTENDER_LEFT_KV = .05; // Volt seconds per meter
        public final static double EXTENDER_RIGHT_KS = .05; // Volts
        public final static double EXTENDER_RIGHT_KV = .05; // Volt seconds per meter
        public final static double ROTATOR_LEFT_KS = .05; // Volts
        public final static double ROTATOR_LEFT_KV = .05; // Volt seconds per meter
        public final static double ROTATOR_RIGHT_KS = .05; // Volts
        public final static double ROTATOR_RIGHT_KV = .05; // Volt seconds per meter


        public final static double LEFT_EXTENDER_HOLD_KP = .005;
        public final static double RIGHT_EXTENDER_HOLD_KP = .005;

        

    }

    public final static class AutonomousConstants{
        public final static double MAX_VEL_METERS_PER_SECOND = 1;
        public final static double MAX_ACC_METERS_PER_SECOND = 0.25;
        /* Tuning constraints:
        - DifferentialDriveVoltageConstraint: If your robot accelerates very slowly then
        it’s possible that the max voltage for this constraint is too low.
        - DifferentialDriveKinematicsConstraint: If your robot ends up at the wrong heading then
        it’s possible that the max drivetrain side speed is too low,
        or that it’s too high. The only way to tell is to tune the max speed and to see what happens.
        - CentripetalAccelerationConstraint: If your robot ends up at the wrong heading then this could be the culprit.
        If your robot doesn’t seem to turn enough then you should increase the max centripetal acceleration,
        but if it seems to go around tight turns to quickly then you should decrease the maximum centripetal acceleration.
        */
    }
    
    public final static class ShooterConstants {
        public final static int LEFT_POWER_WHEEL = 11;  // 11
        public final static int RIGHT_POWER_WHEEL = 12; // 12 
        public final static int AIM_WHEEL = 13;  // 13

        public final static double POWER_WHEEL_KP = 0.0001;
        public final static double POWER_WHEEL_KI = 0;
        public final static double POWER_WHEEL_KD = 0;
        public final static double AIM_WHEEL_KP = 0.0001;
        public final static double AIM_WHEEL_KI = 0;
        public final static double AIM_WHEEL_KD = 0;

        public final static double AIM_WHEEL_TOLERANCE = 100;
        public final static double POWER_WHEEL_TOLERANCE = 100;

        

        public final static boolean LEFT_POWER_WHEEL_INVERTED = true;
        public final static boolean RIGHT_POWER_WHEEL_INVERTED = false;

        public final static double POWER_KS = 0;
        public final static double POWER_KV = 0;
        public final static double POWER_KA = 0;

        public final static double AIM_KS = 0;
        public final static double AIM_KV = 0;
        public final static double AIM_KA = 0;

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

