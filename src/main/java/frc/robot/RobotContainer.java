// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem m_driveSubsystem =  new DriveSubsystem();
  private final XboxController m_Joystick = new XboxController(Constants.JoystickConstants.mainJoystick);
  private SendableChooser<Command> autoSendableChooser = new SendableChooser<Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoSendableChooser.setDefaultOption("test auto", new FollowTrajectory(m_driveSubsystem, "Unnamed"));
    autoSendableChooser.addOption("bad auto", new FollowTrajectory(m_driveSubsystem, "Unnamed_0"));
    SmartDashboard.putData(autoSendableChooser);
    configureButtonBindings();
    //m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.arcadeDrive(() -> m_Joystick.getLeftY(), () -> m_Joystick.getRightX()), m_driveSubsystem));
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
  public Command getAutonomousCommand() {

     // Follow path, then cut voltage to motors (stop)

    return autoSendableChooser.getSelected().andThen(() -> m_driveSubsystem.tankDriveVolts(0.0, 0.0));
  }
  
  public void resetDrive() {
    //m_driveSubsystem.resetOdometry(new Pose2d());
  }
}
