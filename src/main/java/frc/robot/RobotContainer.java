// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FollowTrajectorySequence;
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

  //Subsystems 
  private final DriveSubsystem driveSubsystem =  new DriveSubsystem();

  private final XboxController joystick = new XboxController(Constants.JoystickConstants.mainJoystick);

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private ShuffleboardTab tab;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    addAutoCommands();
    configureButtonBindings();

    tab = Shuffleboard.getTab("Commands");
  }

  public void addAutoCommands() {
    autoChooser.setDefaultOption("AutoTest", new FollowTrajectorySequence(driveSubsystem));
    autoChooser.addOption("Path_1", FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "Unnamed_0"));
    autoChooser.addOption("Path_2", FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "Unnamed"));
    tab.add("AutoCommand", new FollowTrajectorySequence(driveSubsystem));

    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Binds drive subsystem to use the left joystick y/right joystick x to control arcade drive

    driveSubsystem.setDefaultCommand(new ArcadeDrive(() -> -joystick.getLeftY(), () -> -joystick.getRightX(), driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println(autoChooser.getSelected().getName());

     // Follow path, then cut voltage to motors (stop)
    return autoChooser.getSelected().andThen(() -> driveSubsystem.tankDriveVolts(0.0, 0.0));
    
  }

}
