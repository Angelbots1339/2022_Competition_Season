package frc.robot.commands.drive;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends RamseteCommand {

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
        new SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV, DriveConstants.KA);    
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    public static FollowTrajectory followTrajectoryFromJSON(DriveSubsystem driveSubsystem, String fileName, String folder) {
        return new FollowTrajectory(driveSubsystem, getTrajectoryFromJSON(folder, fileName));
    }

    /**
     * For testing PID/Ramsete controller. Not for use in final robot
     * @param driveSubsystem
     * @param fileName
     * @return
     */
    public static RamseteCommand TestFollowTrajectory(DriveSubsystem driveSubsystem, String fileName, String folder) {

        var leftController = new PIDController(DriveConstants.LEFT_KP + 0.5, 0, 0);
        var rightController = new PIDController(DriveConstants.RIGHT_KP + 0.5, 0, 0);

        RamseteController m_disabledRamsete = new RamseteController();
        m_disabledRamsete.setEnabled(true);
        RamseteCommand ramseteCommand = new RamseteCommand(
                getTrajectoryFromJSON(folder, fileName),
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

        //ShuffleboardTab tab = Shuffleboard.getTab("Test");

        // Graph these together to tune P value
        // tab.addNumber(new StringBuffer("LeftActual").append(fileName).toString(), () -> driveSubsystem.getWheelSpeeds().leftMetersPerSecond);
        // tab.addNumber(new StringBuffer("RightActual").append(fileName).toString(), () -> driveSubsystem.getWheelSpeeds().rightMetersPerSecond);
        // tab.addNumber(new StringBuffer("LeftIdeal").append(fileName).toString(), () -> leftController.getSetpoint());
        // tab.addNumber(new StringBuffer("RightIdeal").append(fileName).toString(), () -> rightController.getSetpoint());

        return ramseteCommand;

    }

    /**
     * 
     * @param prefix DEPRECEATED: Will not use parameter. Folder to grab paths from, e.g. "output/"
     * @param pathWeeverFileName Name of path, e.g. "1Ball"
     * @return
     */
    private static Trajectory getTrajectoryFromJSON(String prefix, String pathWeeverFileName) {

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
