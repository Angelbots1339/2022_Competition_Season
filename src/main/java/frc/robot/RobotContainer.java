// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem m_driveSubsystem;
  private final XboxController m_Joystick = new XboxController(Constants.JoystickConstants.mainJoystick);
  private final Timer timer = new Timer();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    timer.start();
    double timerInit = timer.get();
    m_driveSubsystem =  new DriveSubsystem();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Binds drive subsystem to use the left joystick y/right joystick x to control arcade drive
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(() -> -m_Joystick.getLeftY(), () -> -m_Joystick.getRightX(), m_driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Trajectory trajectory) {
    FollowTrajectory pathFollowCommand = new FollowTrajectory(m_driveSubsystem, trajectory);

     // Follow path, then cut voltage to motors (stop)
    return pathFollowCommand.andThen(() -> m_driveSubsystem.tankDriveVolts(0.0, 0.0));
  }
  
  public void resetDrive() {
    //m_driveSubsystem.resetOdometry(new Pose2d());
  }

  public double getTime() {
    return timer.get();
  }
}
