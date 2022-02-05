// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


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

    public final static class JoystickConstants{

         public final static int mainJoystick = 0;
         //public final static int secondaryJoystick = 0; //This is optional


    

        public final static int buttonA = 1;
        public final static int buttonB = 2;
        public final static int buttonX = 3;
        public final static int buttonY = 4;
        public final static int leftBumper = 5;
        public final static int rightBumper = 6;
        public final static int leftMenuButton = 7;
        public final static int rightMenuButton = 8;
        public final static int leftJoystickButton = 9;
        public final static int rightJoystickButton = 10;
        
        

    }
    /**
     * All length units in meters 
     */
    public final static class DriveConstants{
        //general 
        public final static double maxDriveOutput = 0.25;
        public final static boolean LOG_DATA = true;

        //Motor ports
        public final static int LEFT_MOTOR_TOP_PORT = 2; 
        public final static int LEFT_MOTOR_FRONT_PORT = 1; 
        public final static int LEFT_MOTOR_BACK_PORT = 3;
        public final static int RIGHT_MOTOR_TOP_PORT = 5; 
        public final static int RIGHT_MOTOR_FRONT_PORT = 4; 
        public final static int RIGHT_MOTOR_BACK_PORT = 6; 
        
        //Drive base values
        public final static double TRACK_WIDTH = Units.inchesToMeters(21.5); // Center of left wheel to center of right wheel
        public final static double WHEEL_DIAMETER = Units.inchesToMeters(3.875);
        public final static double WHEEL_ROT_PER_MOTOR_ROT = 1/6.67;
        public final static double CLICKS_PER_ROT = 2048;
        public final static DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
        public final static boolean RIGHT_INVERTED = true;
        public final static boolean LEFT_INVERTED = false;
        public final static double CLICKS_TO_METERS = 1 / CLICKS_PER_ROT
                * WHEEL_ROT_PER_MOTOR_ROT * WHEEL_DIAMETER * Math.PI;
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
        public final static int LOADER_PORT = 10; // 10
        public final static double MAX_INDEXER_SPEED = 0.5;
        public final static double MAX_INTAKE_SPEED = 0.6;
        public final static double MAX_LOADER_SPEED = 0.6;

        public final static boolean INDEXER_LEFT_INVERSE = false;
        public final static boolean INDEXER_RIGHT_INVERSE = true;
        public final static boolean LOADER_INVERSE = false;
        public final static boolean INTAKE_INVERSE = false;
        
        public final static int SERVO_RIGHT_PORT = 0;
        public final static int SERVO_LEFT_PORT = 1;

        public final static int COLOR_SENSOR_PROXIMITY_THRESHOLD = 1000; // 0 to 2047

        
        // TODO Fix color sensor ports
        public final static int COLOR_SENSOR_HIGH_PORT = -1;
        public final static int COLOR_SENSOR_LOW_PORT = -1;




    }

    public static final class ClimberConstants{
        public static final int ROTATOR_LEFT_PORT = 16;  // 16 
        public static final int ROTATOR_RIGHT_PORT = 17; // 17
        public static final int EXTENDER_LEFT_PORT = 14; // 14
        public static final int EXTENDER_RIGHT_PORT = 15; // 15
        public static final boolean ROTATOR_LEFT_INVERTED = false;
        public static final boolean ROTATOR_RIGHT_INVERTED = true;
        public static final boolean EXTENDER_LEFT_INVERTED = false;
        public static final boolean EXTENDER_RIGHT_INVERTED = true;
        public static final double ROTATOR_SPEED = 0.5;
        public static final double EXTENDER_SPEED = 0.5;
        
        public static final int ROTATOR_LEFT_LIMIT_PORT = -1;
        public static final int ROTATOR_RIGHT_LIMIT_PORT = -1;
        public static final double LIMIT_SWITCH_DEBOUNCE_TIME = 0.02;

        public static final double SLACK_LENGTH = Units.inchesToMeters(60);
       
        private static final double MOTOR_ROT_PER_SPOOL_ROT = 30/1;
        private static final double MOTOR_ROT_PER_ARM_ROT = 30/1;
        private static final double SPOOL_CIRCUM = Units.inchesToMeters(1) * 2 * Math.PI;
        public static final double LENGTH_PER_CLICK = SPOOL_CIRCUM * MOTOR_ROT_PER_SPOOL_ROT / DriveConstants.CLICKS_PER_ROT;
        
        
        public final static double GET_DEGREES_FROM_CLICKS(double Clicks){
            return Math.IEEEremainder(Clicks / DriveConstants.CLICKS_PER_ROT * MOTOR_ROT_PER_ARM_ROT * 360, 360);

        }


        public final static double EXTENDER_TOLERANCE = 1;
        public final static double EXTENDER_P_TOLERANCE = 1;
        public final static double ROTATOR_ANGLE_P_TOLERANCE = 1; // error angle at which it switches to pid
        public final static double ROTATOR_ANGLE_TOLERANCE = 1; // error angle at which it stops
        public final static double EXTENDER_KP = 1;
        public final static double ROTATOR_KP = 0.01;

    }

    public final static class AutonomousConstants{
        public final static double maxVelocityMetersPerSecond = 1;
        public final static double maxAccelerationMetersPerSecondSq = 0.25;
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

        public final static double POWER_WHEEL_KP = 0.001;
        public final static double POWER_WHEEL_KI = 0;
        public final static double POWER_WHEEL_KD = 0;
        public final static double AIM_WHEEL_KP = 0.001;
        public final static double AIM_WHEEL_KI = 0;
        public final static double AIM_WHEEL_KD = 0;

        public final static double AIM_WHEEL_TOLERANCE = 100;
        public final static double POWER_WHEEL_TOLERANCE = 100;

        

        public final static boolean LEFT_POWER_WHEEL_INVERTED = false;
        public final static boolean RIGHT_POWER_WHEEL_INVERTED = true;

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

