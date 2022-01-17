// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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


        
        public final static int leftJoystickY = 1;
        public final static int leftJoystickX = 2;
        public final static int rightJoystickX = 4;
        public final static int rightJoystickY = 4;
        public final static int dpadHorizontal = 5;
        public final static int dpadVertical = 6;



        public final static int buttonX = 1;
        public final static int buttonA = 2;
        public final static int buttonB = 3;
        public final static int buttonY = 4;
        public final static int leftBumper = 5; //TODO Actually put in ports for the joystick
        public final static int rightBumper = 6;
        public final static int leftTrigger = 7;
        public final static int rightTrigger = 8;
        public final static int leftJoystickButton = 0;
        public final static int rightJoystickButton = 0;
        
        

    }
    /**
     * All length units in meters 
     */
    public final static class DriveConstants{
        //general 
        public final static double maxDriveOutput = 1;

        //Motor ports
        public final static int leftMotorTopPort = 2; 
        public final static int leftMotorFrontPort = 1; 
        public final static int leftMotorBackPort = 3; 
        public final static int rightMotorTopPort = 5; 
        public final static int rightMotorFrontPort = 4; 
        public final static int rightMotorBackPort = 6; 
        
        //Drive base values
        public final static double trackWidth = Units.inchesToMeters(21.5);
        public final static double wheelDiameter = Units.inchesToMeters(4);
        public final static double wheelRotPerMotorRot = 1/6.67;
        public final static double falcon500ClicksPerRot = 2048;
        public static final DifferentialDriveKinematics m_DriveKinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);

        //PID 
        public final static double leftKP = 2.1258;
        public final static double rightKP = 2.1258;
        public final static double slewRateLimitFwd = 2;
        public final static double slewRateLimitRot = 2;

        //Motion profiling
        public final static double ks = 0.5221; // Volts
        public final static double kv = 2.1103;  // Volts * Seconds / Meters
        public final static double ka = 0.11835; // Volts * Seconds^2 / Meters
        
        /* Checking kV:
        kV = voltage / free speed (meters per second)
        free speed = free speed of the motor times the wheel circumference divided by the gear reduction
        kV = 12v / (5380rpm / 60s * 0.1016m * pi / 6.67)
        kV (theoretical) ~ 2.797
        */

        public static final boolean debug = true;

    }
    public final static class AutonomousConstants{
        public final static double maxVelocityMetersPerSecond = 1;
        public final static double maxAccelerationMetersPerSecondSq = 0.5;

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
