package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends RamseteCommand {
    private final DriveSubsystem driveSubsystem;
    private static Pose2d zeroPose = new Pose2d();
    private static DifferentialDriveVoltageConstraint voltageConstraint;
    private static TrajectoryConfig config = new TrajectoryConfig(AutonomousConstants.maxVelocityMetersPerSecond,
            AutonomousConstants.maxAccelerationMetersPerSecondSq);
    private final SimpleMotorFeedforward simpleMotorFeedforward;
    private Trajectory trajectory;

    /**
     * Create a RamseteCommand to follow a given trajectory
     * 
     * @param m_driveSubsystem Drive subsystem for sensors & dependency injection
     * @param trajectory       Trajectory to follow. See {@link #getAutoTrajectory}
     *                         for autonomous trajectory
     */
    public FollowTrajectory(DriveSubsystem m_driveSubsystem, Trajectory trajectory) {
        super(trajectory,
                m_driveSubsystem::getPose, new RamseteController(DriveConstants.KB, DriveConstants.ZETA),
                m_driveSubsystem.getFeedforward(), DriveConstants.DRIVE_KINEMATICS,
                m_driveSubsystem::getWheelSpeeds,
                m_driveSubsystem.getLeftPid(), m_driveSubsystem.getRightPid(),
                m_driveSubsystem::tankDriveVolts,
                m_driveSubsystem);
        this.driveSubsystem = m_driveSubsystem;
        addRequirements(m_driveSubsystem);
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
        driveSubsystem.resetPose2D(trajectory.getInitialPose());
    }


    public static FollowTrajectory followTrajectoryFromJSON(DriveSubsystem driveSubsystem, String fileName) {
        return new FollowTrajectory(driveSubsystem, getTrajectoryFromJSON(fileName));
    }

    private static Trajectory getTrajectoryFromJSON(String pathWeeverFileName) {

        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + pathWeeverFileName + ".wpilib.json");
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathWeeverFileName, ex.getStackTrace());
        }
        return trajectory;
    }

    /**
     * @return Trajectory to be followed for the auto routine
     */
    public static Trajectory getAutoTrajectory() {
        System.out.println("Called getAutoTrajectory");
        // Draw an 's' curve
        Trajectory trajectory = TrajectoryGenerator
                .generateTrajectory(zeroPose, List.of(
                        new Translation2d(1.5, 0.5)),
                        new Pose2d(2, 2, Rotation2d.fromDegrees(90)),
                        config);

        return trajectory;
    }
    public Pose2d getStartPose2d() {
        return trajectory.getInitialPose();
    }

}
