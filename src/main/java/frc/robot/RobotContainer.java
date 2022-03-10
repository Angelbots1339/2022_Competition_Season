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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoSequences;
import frc.robot.commands.Shoot;
import frc.robot.commands.ToggleCamera;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.EjectBalls;
import frc.robot.commands.climber.ArmsToSetpoints;
import frc.robot.commands.climber.AutoClimb;
import frc.robot.commands.climber.ManualArms;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.IdleShooter;
import frc.robot.commands.RejectBall;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final static DriveSubsystem driveSubsystem =  new DriveSubsystem();
  private final static IntakeSubsystem intakeSubsystem =  new IntakeSubsystem();
  private final static ClimbingSubsystem climbingSubsystem =  new ClimbingSubsystem();
  private final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final static LoaderSubsystem loaderSubsystem = new LoaderSubsystem();

  private static final AutoSequences autos = new AutoSequences(driveSubsystem, intakeSubsystem, loaderSubsystem, shooterSubsystem);

  private final XboxController joystick = new XboxController(Constants.JoystickConstants.MAIN_JOYSTICK);

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private ShuffleboardTab tab = Shuffleboard.getTab("RobotContainer");

  private NetworkTableEntry isTeamRed = tab.add("rejectRed", false).getEntry();
  
  private boolean driveMode = true;

  private boolean isDriveReversed = DriveConstants.USE_LIMELIGHT_FIRST;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    addAutoCommands();
    configureButtonBindings();
    driveSubsystem.resetOdometry(new Pose2d());

  }

  public void resetOdometry() {
    driveSubsystem.resetOdometry(new Pose2d());
  }

  /**
   * Call in teleop init
   */
  public void resetArms() {
    climbingSubsystem.reset(true);
  }

  /**
   * Populate auto chooser in SmartDashboard with auto commands
   */
  public void addAutoCommands() {
    // Sequence
    autos.forEach((cmd) -> autoChooser.addOption(cmd.toString(), cmd));

    // Single
    // autoChooser.addOption("Forward", FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "Forward"));
    // autoChooser.addOption("TurnLeft", FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "TurnLeft"));
    // autoChooser.addOption("2Meter", FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "2Meter"));
    
   
   
    tab.add(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* DRIVING */

    // Invert drive when using rear camera
    DoubleSupplier fwd = () -> (isDriveReversed? 1 : -1) * joystick.getLeftY();
    DoubleSupplier rot = () -> -joystick.getRightX()  * DriveConstants.ROT_SCALE;

    // Set drive default command to left Y (speed) right X (turn)
    driveSubsystem.setDefaultCommand(new ArcadeDrive(fwd, rot, driveSubsystem));

    // Feed drive watchdog when idle
    Command stopDrive = new RunCommand(() -> driveSubsystem.disable(), driveSubsystem);
    Command stopShooter = new RunCommand(() -> shooterSubsystem.disable(), shooterSubsystem);
    Command stopIntake = new RunCommand(() -> intakeSubsystem.disable(), intakeSubsystem);
    Command stopLoader = new RunCommand(() -> loaderSubsystem.disable(), loaderSubsystem);
    Command stopToClimb = new ParallelCommandGroup(stopDrive, stopShooter, stopIntake, stopLoader);


    /* CLIMBING */

    // Bind extension to left axis, rotation to right axis
    DoubleSupplier extension = () -> (joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis()) * ClimberConstants.MAX_EXTENDER_VOLTS;
    DoubleSupplier rotation = () -> -joystick.getRightY() * ClimberConstants.MAX_ROTATOR_VOLTS;
    

    climbingSubsystem.setDefaultCommand(new ManualArms(climbingSubsystem, extension, () -> 0));

    // Right Menu to toggle between driving and climbing
    new JoystickButton(joystick, RIGHT_MENU_BUTTON).toggleWhenPressed(new ManualArms(climbingSubsystem, extension, rotation))
      .toggleWhenPressed(stopToClimb)
      .toggleWhenPressed(new InstantCommand(() -> {driveMode = !driveMode;}));

    // Start auto climb when left menu button pressed, and release to stop. Press X to proceed
    new JoystickButton(joystick, LEFT_MENU_BUTTON).toggleWhenPressed(new AutoClimb(climbingSubsystem, () -> joystick.getXButton()))
      .toggleWhenPressed(stopToClimb)
      .toggleWhenPressed(new InstantCommand(() -> {driveMode = false;}));

    // Go to initial climb setpoint when b button is pressed and not in drive mode
    new JoystickButton(joystick, BUTTON_B).whenPressed(new ConditionalCommand(new InstantCommand(), new ArmsToSetpoints(climbingSubsystem, .6, 0), () -> !driveMode));


    /* SHOOTING */

    // Shoot high when Y button is pressed
    new JoystickButton(joystick, BUTTON_Y).whileHeld(new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem, ShooterConstants.SHOOTER_PROFILE_HIGH, 
        () -> joystick.getRightStickButton()));

    // Shoot low when A button is pressed
    new JoystickButton(joystick, BUTTON_A).whileHeld(new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem, ShooterConstants.SHOOTER_PROFILE_LOW, 
        () -> joystick.getRightStickButton()));

    shooterSubsystem.setDefaultCommand(new IdleShooter(shooterSubsystem));


    /* INTAKE */

    // Run Intake-in while the left bumper is held
    new JoystickButton(joystick, LEFT_BUMPER).whenHeld(new RunIntake(intakeSubsystem, loaderSubsystem));

    // Run reverse intake when right bumper is pressed
    new JoystickButton(joystick, RIGHT_BUMPER).whenHeld(new EjectBalls(intakeSubsystem, loaderSubsystem));

    loaderSubsystem.setDefaultCommand(new RejectBall(loaderSubsystem, intakeSubsystem, () -> isTeamRed.getBoolean(false)));
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //driveSubsystem.resetOdometry(new Pose2d());
    //System.err.println(autoChooser.getSelected().getName());

     // Follow path, then cut voltage to motors (stop)
     

    return autoChooser.getSelected().andThen(driveSubsystem::disable).andThen(shooterSubsystem::disable).andThen(intakeSubsystem::disable).andThen(loaderSubsystem::disable);
    
  }

  public void testModeRunArms() {
    climbingSubsystem.setTestExtenderPercent(joystick.getLeftY() * 0.4, joystick.getRightY() *0.4);
    climbingSubsystem.setTestRotatorPercent(-joystick.getLeftTriggerAxis() * 0.1, -joystick.getRightTriggerAxis() * 0.1);
    if(joystick.getXButton()) {
      climbingSubsystem.reset(false);
    }
    if(joystick.getBButton()) {
      climbingSubsystem.reset(true);
    }
  }

  public void setDriveMode() {
    driveMode = false;
  }
}
