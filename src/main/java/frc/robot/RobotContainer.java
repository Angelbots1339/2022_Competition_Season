// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FollowTrajectorySequence;
import frc.robot.commands.RevShooter;
import frc.robot.commands.ToggleCamera;
import frc.robot.commands.Intake.LoadShooter;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.ejectBalls;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.ShooterProfiles;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.JoystickConstants.*;

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
  private final IntakeSubsystem intakeSubsystem =  new IntakeSubsystem();
  private final ClimbingSubsystem climbingSubsystem =  new ClimbingSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();

  private final XboxController joystick = new XboxController(Constants.JoystickConstants.mainJoystick);

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private ShuffleboardTab tab = Shuffleboard.getTab("RobotContainer");

  private boolean isDriveReversed;

  private NetworkTableEntry powerWheelRPM = tab.add("Max Speed", 100)
      .getEntry();
  private NetworkTableEntry aimWheelRPM = tab.add("Max Speed", 100)
      .getEntry();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    addAutoCommands();
    configureButtonBindings();
    driveSubsystem.resetOdometry(new Pose2d());
    intakeSubsystem.deployIntake();

    
  }

  public void resetOdometry() {
    driveSubsystem.resetOdometry(new Pose2d());
  }

  /**
   * Populate auto chooser in SmartDashboard with auto commands
   */
  public void addAutoCommands() {
    // Sequence
    autoChooser.setDefaultOption("AutoTestPathWeever", new FollowTrajectorySequence(driveSubsystem));

    // Single
    autoChooser.addOption("Forward", FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "Forward"));
    autoChooser.addOption("TurnLeft", FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "TurnLeft"));
    autoChooser.addOption("2Meter", FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "2Meter"));

    
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Invert drive when using rear camera
    DoubleSupplier fwd = () -> (isDriveReversed? -1 : 1) * joystick.getLeftY();
    DoubleSupplier rot = () -> -joystick.getRightX();
    driveSubsystem.setDefaultCommand(new ArcadeDrive(fwd, rot, driveSubsystem));

    Command climbCommand = new RunCommand(() -> {
      climbingSubsystem.setRotationSpeed(() -> joystick.getRightY()); 
      climbingSubsystem.setExtensionSpeed(() -> joystick.getLeftY());
    } , climbingSubsystem);
    Command stopDrive = new RunCommand(() -> driveSubsystem.tankDriveVolts(0, 0), driveSubsystem);

    new JoystickButton(joystick, BUTTON_A).whenHeld(climbCommand).whenHeld(stopDrive);
    
    // Toggle cameras & drive when B is pressed
    new JoystickButton(joystick, BUTTON_B).toggleWhenPressed(new ToggleCamera(
        (boolean isDriveReversed) -> this.isDriveReversed = isDriveReversed));

    // Run Intake while the left bumper is held
    new JoystickButton(joystick, LEFT_BUMPER).whenHeld(new RunIntake(intakeSubsystem, loaderSubsystem));

    // Run feeder when the Y button is pressed
    new JoystickButton(joystick, BUTTON_Y).whenHeld(new LoadShooter(loaderSubsystem));

    // Run eject balls when menu button is pressed
    new JoystickButton(joystick, LEFT_MENU_BUTTON).whenHeld(new ejectBalls(intakeSubsystem, loaderSubsystem));

    // When the right bumper is held down, the flywheel on the shooter will spin up to set speed
    new JoystickButton(joystick, RIGHT_BUMPER).whenHeld(new RevShooter(shooterSubsystem, new ShooterProfiles(() -> powerWheelRPM.getDouble(0), () -> aimWheelRPM.getDouble(0)), joystick));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    driveSubsystem.resetOdometry(new Pose2d());
    System.out.println(autoChooser.getSelected().getName());

     // Follow path, then cut voltage to motors (stop)
    return autoChooser.getSelected().andThen(() -> driveSubsystem.tankDriveVolts(0.0, 0.0));
    
  }

}
