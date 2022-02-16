// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FollowTrajectorySequence;
import frc.robot.commands.Shoot;
import frc.robot.commands.ToggleCamera;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.ejectBalls;
import frc.robot.commands.climber.GoToBar;
import frc.robot.commands.climber.ManualArms;
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
import frc.robot.Constants.ClimberConstants;

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

  private NetworkTableEntry powerShootSpeed = tab.add("Front Power Shoot Speed", 0)
      .getEntry();
  private NetworkTableEntry aimShootSpeed = tab.add("Front Aim Shoot Speed", 0)
      .getEntry();
    

  private boolean isDriveReversed;
  private ShooterProfiles testProfiles;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    addAutoCommands();
    configureButtonBindings();
    driveSubsystem.resetOdometry(new Pose2d());
    tab.addNumber("Power Wheel Rpm", () -> shooterSubsystem.getPowerRPM());
    tab.addNumber("Aim Wheel Rpm", () -> shooterSubsystem.getAimRPM());
    

    
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
    DoubleSupplier rot = () -> -joystick.getRightX()  * DriveConstants.ROT_SCALE;
    driveSubsystem.setDefaultCommand(new ArcadeDrive(fwd, rot, driveSubsystem));

    // Feed drive watchdog when idle
    Command stopDrive = new RunCommand(() -> driveSubsystem.tankDriveVolts(0, 0), driveSubsystem);
    // Bind extension to left axis, rotation to right axis

    DoubleSupplier extension = () -> (joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis()) * ClimberConstants.MAX_EXTENDER_VOLTS;
    DoubleSupplier rotation = () -> -joystick.getRightY() * ClimberConstants.MAX_ROTATOR_VOLTS;
    BooleanSupplier proceed = () -> joystick.getRawButtonPressed(LEFT_MENU_BUTTON);

    // Right Menu to toggle between driving and climbing
    new JoystickButton(joystick, RIGHT_MENU_BUTTON).toggleWhenPressed(new ManualArms(climbingSubsystem, extension, rotation)).toggleWhenPressed(stopDrive);
    
    ManualArms manualArms = new ManualArms(climbingSubsystem, extension, rotation);

    climbingSubsystem.setDefaultCommand(new ManualArms(climbingSubsystem, extension, () -> 0));

    
    new JoystickButton(joystick, RIGHT_MENU_BUTTON).whenPressed(new GoToBar(climbingSubsystem, manualArms, proceed));

    // Toggle cameras & drive when B is pressed
    new JoystickButton(joystick, BUTTON_B).toggleWhenPressed(new ToggleCamera(
       (boolean isDriveReversed) -> this.isDriveReversed = isDriveReversed));

    // Run Intake-in while the left bumper is held
    new JoystickButton(joystick, LEFT_BUMPER).whenHeld(new RunIntake(intakeSubsystem, loaderSubsystem));

    // Shoot high when Y button is pressed

    new JoystickButton(joystick, BUTTON_Y).whileHeld(new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem, ShooterConstants.SHOOTER_PROFILE_HIGH));

    // Shoot low when A button is pressed

    new JoystickButton(joystick, BUTTON_A).whileHeld(new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem, ShooterConstants.SHOOTER_PROFILE_LOW));


    // Run reverse intake when right bumper is pressed
    new JoystickButton(joystick, RIGHT_BUMPER).whenHeld(new ejectBalls(intakeSubsystem, loaderSubsystem));


   
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

  public void printColorSensor() {
    SmartDashboard.putBoolean("Color Sensor Tripped", intakeSubsystem.isBallLow());
  }

}
