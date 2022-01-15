// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final XboxController m_Joystick = new XboxController(Constants.JoystickConstants.mainJoystick);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureButtonBindings();

    // Binds drive subsystem to use the left joystick to control arcade drive
    m_driveSubsystem.setDefaultCommand(new RunCommand(
        () -> m_driveSubsystem.arcadeDrive(() -> -m_Joystick.getLeftY(),
            () -> m_Joystick.getRightX()),
        m_driveSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        m_driveSubsystem.getKinematics(), 10);

    // Simple path following auto
    TrajectoryConfig config = new TrajectoryConfig(AutonomousConstants.maxVelocityMetersPerSecond,
        AutonomousConstants.maxAccelerationMetersPerSecondSq);
    config.setKinematics(m_driveSubsystem.getKinematics()).addConstraint(voltageConstraint);

    // simple trajectory that moves 1 meter forward
    Trajectory trajectory = TrajectoryGenerator
        .generateTrajectory(new Pose2d(), List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)),
          new Pose2d(3, 0, new Rotation2d()),
           config);

    RamseteCommand command = new RamseteCommand(trajectory,
        m_driveSubsystem::getPose, new RamseteController(2, 0.7),
        m_driveSubsystem.getFeedforward(), m_driveSubsystem.getKinematics(),
        m_driveSubsystem::getWheelSpeeds,
        m_driveSubsystem.getLeftPid(), m_driveSubsystem.getRightPid(),
        m_driveSubsystem::tankDriveVolts,
        m_driveSubsystem);

    m_driveSubsystem.resetOdometry(trajectory.getInitialPose());

    return command.andThen(() -> m_driveSubsystem.tankDriveVolts(0.0, 0.0));

  }

  public void resetDrive() {
    m_driveSubsystem.resetOdometry(new Pose2d());
  }
}
