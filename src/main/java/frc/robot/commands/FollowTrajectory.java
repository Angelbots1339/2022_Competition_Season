package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * 
 */
public class FollowTrajectory extends RamseteCommand{
    private final DriveSubsystem m_driveSubsystem;
    private static Pose2d zeroPose = new Pose2d();
    private static DifferentialDriveVoltageConstraint voltageConstraint;
    private static TrajectoryConfig config;
    private final SimpleMotorFeedforward simpleMotorFeedforward;

    /**
     * Create a RamesteCommand to follow a given trajectory
     * @param m_driveSubsystem Drive subsytem for sensors & dependancy injection
     * @param trajectory Trajectory to follow. See {@link #getAutoTrajectory} for autonomous trajectory
     */
    public FollowTrajectory(DriveSubsystem m_driveSubsystem, Trajectory trajectory) {
        // TODO b & zeta should be constants
        super(trajectory,
            m_driveSubsystem::getPose, new RamseteController(2, 0.7),
            m_driveSubsystem.getFeedforward(), DriveConstants.m_DriveKinematics,
            m_driveSubsystem::getWheelSpeeds,
            m_driveSubsystem.getLeftPid(), m_driveSubsystem.getRightPid(),
            m_driveSubsystem::tankDriveVolts,
            m_driveSubsystem
        );
        this.m_driveSubsystem = m_driveSubsystem;
        addRequirements(m_driveSubsystem);
        simpleMotorFeedforward = new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka);
        // Constrain the max voltage to 10
        voltageConstraint = new DifferentialDriveVoltageConstraint(simpleMotorFeedforward, DriveConstants.m_DriveKinematics, 10);
        TrajectoryConfig config = new TrajectoryConfig(AutonomousConstants.maxVelocityMetersPerSecond,
            AutonomousConstants.maxAccelerationMetersPerSecondSq);
        config.setKinematics(DriveConstants.m_DriveKinematics).addConstraint(voltageConstraint);
    }

    @Override
    public void initialize() {
        super.initialize();
        m_driveSubsystem.resetOdometry(zeroPose);
    }

    /**
     * @return Trajectory to be followed for the auto routine
     */
    public static Trajectory getAutoTrajectory() {
        System.out.println("Called getAutoTrajectoy");
        // Draw an 's' curve
        Trajectory trajectory = TrajectoryGenerator
            .generateTrajectory(zeroPose, List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d()),
            config);

        return trajectory;
    }
}
