package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends RamseteCommand {

    private static DifferentialDriveVoltageConstraint voltageConstraint;
    private static TrajectoryConfig config = new TrajectoryConfig(AutonomousConstants.MAX_VEL_METERS_PER_SECOND,
            AutonomousConstants.MAX_ACC_METERS_PER_SECOND);
    private final SimpleMotorFeedforward simpleMotorFeedforward;
    private Trajectory trajectory;

    /**
     * Create a RamseteCommand to follow a given trajectory
     * 
     * @param driveSubsystem Drive subsystem for sensors & dependency injection
     * @param trajectory     Trajectory to follow. See {@link #getAutoTrajectory}
     *                       for autonomous trajectory
     */
    public FollowTrajectory(DriveSubsystem driveSubsystem, Trajectory trajectory) {
        super(trajectory,
                driveSubsystem::getPose, new RamseteController(DriveConstants.KB, DriveConstants.ZETA),
                driveSubsystem.getFeedforward(), DriveConstants.DRIVE_KINEMATICS,
                driveSubsystem::getWheelSpeeds,
                driveSubsystem.getLeftPid(), driveSubsystem.getRightPid(),
                driveSubsystem::tankDriveVolts,
                driveSubsystem);
        

        addRequirements(driveSubsystem);
        simpleMotorFeedforward = new SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV, DriveConstants.KA);
        // Constrain the max voltage to 10
        voltageConstraint = new DifferentialDriveVoltageConstraint(simpleMotorFeedforward,
                DriveConstants.DRIVE_KINEMATICS, 10);
        config.setKinematics(DriveConstants.DRIVE_KINEMATICS).addConstraint(voltageConstraint);
        

        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    public static FollowTrajectory followTrajectoryFromJSON(DriveSubsystem driveSubsystem, String fileName) {
        return new FollowTrajectory(driveSubsystem, getTrajectoryFromJSON(fileName));
    }

    /**
     * For testing PID/Ramsete controller. Not for use in final robot
     * @param driveSubsystem
     * @param fileName
     * @return
     */
    public static RamseteCommand TestFollowTrajectory(DriveSubsystem driveSubsystem, String fileName) {

        var leftController = new PIDController(DriveConstants.LEFT_KP + 0.5, 0, 0);
        var rightController = new PIDController(DriveConstants.RIGHT_KP + 0.5, 0, 0);

        RamseteController m_disabledRamsete = new RamseteController();
        m_disabledRamsete.setEnabled(true);
        RamseteCommand ramseteCommand = new RamseteCommand(
                getTrajectoryFromJSON(fileName),
                driveSubsystem::getPose,
                m_disabledRamsete, // Pass inm disabledRamsete here
                driveSubsystem.getFeedforward(),
                Constants.DriveConstants.DRIVE_KINEMATICS,
                driveSubsystem::getWheelSpeeds,
                leftController,
                rightController,
                // RamseteCommand passes volts to the callback
                (leftVolts, rightVolts) -> {
                    driveSubsystem.tankDriveVolts(leftVolts, rightVolts);
                },
                driveSubsystem);

        ShuffleboardTab tab = Shuffleboard.getTab("Test");

        // Graph these together to tune P value
        tab.addNumber(new StringBuffer("LeftActual").append(fileName).toString(), () -> driveSubsystem.getWheelSpeeds().leftMetersPerSecond);
        tab.addNumber(new StringBuffer("RightActual").append(fileName).toString(), () -> driveSubsystem.getWheelSpeeds().rightMetersPerSecond);
        tab.addNumber(new StringBuffer("LeftIdeal").append(fileName).toString(), () -> leftController.getSetpoint());
        tab.addNumber(new StringBuffer("RightIdeal").append(fileName).toString(), () -> rightController.getSetpoint());

        return ramseteCommand;

    }

    private static Trajectory getTrajectoryFromJSON(String pathWeeverFileName) {

        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
                    .resolve("output/" + pathWeeverFileName + ".wpilib.json");
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathWeeverFileName, ex.getStackTrace());
        }
        return trajectory;
    }

    public Pose2d getStartPose2d() {
        return trajectory.getInitialPose();
    }

}
