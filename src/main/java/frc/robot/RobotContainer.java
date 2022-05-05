// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Console;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.ClearDrivingFaults;
import frc.robot.commands.drive.TargetBall;
import frc.robot.commands.auto.AutoSequences;
import frc.robot.commands.climber.ArmsToSetpoints;
import frc.robot.commands.climber.AutoClimb;
import frc.robot.commands.climber.ClearClimbingFaults;
import frc.robot.commands.climber.ManualArms;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.EjectBalls;
import frc.robot.commands.intake.RejectBall;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Logging;
import frc.robot.utils.NetworkTablesHelper;
import frc.robot.utils.Targeting;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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

  // Subsystems
  private final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final static ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();
  private final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final static LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
  private final static CandleSubsystem candleSubsystem = new CandleSubsystem();

  private final XboxController joystick = new XboxController(Constants.JoystickConstants.MAIN_JOYSTICK);

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public static ShuffleboardTab tab = Shuffleboard.getTab("RobotContainer");
  private static boolean isTeamRed = false;
  //private static BooleanSupplier isTeamRed = () -> false;//() -> NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false);
  
  private boolean driveMode = true;
  private static boolean rejectBalls = true;
  private static final AutoSequences autos = new AutoSequences(driveSubsystem, intakeSubsystem, loaderSubsystem,
      shooterSubsystem, rejectBalls);
  
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

  public void setPipeline(){

    Targeting.setPipeline(isTeamRed ? 0 : 1);

  }

  public void setToDefaultAnimation(){

    candleSubsystem.setToDefaultAnimation();

  }

  /**
   * Populate auto chooser in SmartDashboard with auto commands
   */
  public void addAutoCommands() {
    // Sequence
    // TODO adding autos overrunning loop times? try timer & speed up code or start new thread
    autos.forEach((cmd) -> autoChooser.addOption(cmd.toString(), cmd));
   
    //tab.addNumber("Camera pipeline", () -> Targeting.getPipeline());
    if(Logging.general) {
      tab.addBoolean("isTEAMred", () -> isTeamRed);
    }
    

    //Test code for turn and arms
    // SmartDashboard.putData("turn 90",new TurnToAngle(driveSubsystem, 90));
    // SmartDashboard.putData("turn -90", new TurnToAngle(driveSubsystem, -90));

    // SmartDashboard.putData("arms up", new PIDArmsToSetpoints(climbingSubsystem, ClimberConstants.EXTENDER_TOP_LIMIT, 0, new ArmSpeeds(0, 0, 1, 1)));
    // SmartDashboard.putData("arms down", new PIDArmsToSetpoints(climbingSubsystem, ClimberConstants.EXTENDER_BOTTOM_LIMIT, 0, new ArmSpeeds(0, 0, 1, 1)));

    // SmartDashboard.putData("BrakeMode", new ArmsToSetpoints(climbingSubsystem, 0, 0, 3, 0, true, true));
    // SmartDashboard.putData("rotator front", new PIDArmsToSetpoints(climbingSubsystem, 0, ClimberConstants.ROTATOR_FRONT_LIMIT_DEG, new ArmSpeeds(10, 10, 0, 0)));
    // SmartDashboard.putData("rotator back", new PIDArmsToSetpoints(climbingSubsystem, 0, ClimberConstants.ROTATOR_BACK_LIMIT_DEG, new ArmSpeeds(10, 10, 0, 0)));

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
    DoubleSupplier fwd = () -> -joystick.getLeftY();
    DoubleSupplier rot = () -> -joystick.getRightX() * DriveConstants.ROT_SCALE;

    // Set drive default command to left Y (speed) right X (turn)
    driveSubsystem.setDefaultCommand(new ArcadeDrive(fwd, rot, driveSubsystem));

    // Target ball when x pressed and not in climb mode.
    // new JoystickButton(joystick, BUTTON_X).whenHeld(new ConditionalCommand(
    //   new TargetBall(driveSubsystem, fwd, rot),
    //   new InstantCommand(), () -> !driveMode));
    // new JoystickButton(joystick, BUTTON_X).whenHeld(new TargetBall(driveSubsystem, fwd, rot));

    // Feed drive watchdog when idle
    Command stopDrive = new RunCommand(() -> driveSubsystem.disable(), driveSubsystem);
    Command stopShooter = new RunCommand(() -> shooterSubsystem.disable(), shooterSubsystem);
    Command stopIntake = new RunCommand(() -> intakeSubsystem.disable(), intakeSubsystem);
    Command stopLoader = new RunCommand(() -> loaderSubsystem.disable(), loaderSubsystem);
    Command stopToClimb = new ParallelCommandGroup(stopDrive, stopShooter, stopIntake, stopLoader);

    /* CLIMBING */

    // Bind extension to left axis, rotation to right axis
    DoubleSupplier extension = () -> (
      joystick.getLeftTriggerAxis() * ClimberConstants.DROP_EXTENDER_VOLTS
    - joystick.getRightTriggerAxis() * ClimberConstants.MANUAL_UP_VOLTS);
    DoubleSupplier rotation = () -> -joystick.getRightY() * ClimberConstants.MAX_ROTATOR_VOLTS;

    // TODO move to schedule in auto
    climbingSubsystem.setDefaultCommand(new ManualArms(climbingSubsystem, extension, () -> 0));

    // Right Menu to toggle between driving and climbing
    new JoystickButton(joystick, RIGHT_MENU_BUTTON)
        .toggleWhenPressed(new ManualArms(climbingSubsystem, extension, rotation))
        .toggleWhenPressed(stopToClimb)
        .toggleWhenPressed(new InstantCommand(() -> {
          driveMode = !driveMode;
        }));

    // Start auto climb when left menu button pressed, and release to stop. Press X
    // to proceed
    new JoystickButton(joystick, LEFT_MENU_BUTTON)
        .toggleWhenPressed(new AutoClimb(climbingSubsystem, () -> joystick.getXButton()))
        .toggleWhenPressed(stopToClimb)
        .toggleWhenPressed(new InstantCommand(() -> {
          driveMode = false;
        }));

    // Go to initial climb setpoint when b button is pressed and not in drive mode
    new JoystickButton(joystick, BUTTON_B).whenPressed(
        new ConditionalCommand(
            new ArmsToSetpoints(climbingSubsystem, 0, 0, 4, 1),
            new ArmsToSetpoints(climbingSubsystem, .6, 0), () -> !driveMode));

    /* SHOOTING */

    // Shoot high when Y button is pressed
    new JoystickButton(joystick, BUTTON_Y).whileHeld(
      new ClearClimbingFaults(climbingSubsystem)
      .andThen(new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem, ShooterConstants.SHOOTER_PROFILE_HIGH,
            isTeamRed)))
    .whenReleased(new ArmsToSetpoints(climbingSubsystem, 0, 0, 4, 1));

    // Shoot low when A button is pressed
    new JoystickButton(joystick, BUTTON_A).whileHeld(
      new ClearClimbingFaults(climbingSubsystem)
      .andThen(new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem, ShooterConstants.SHOOTER_PROFILE_LOW,
            isTeamRed)))
    .whenReleased(new ArmsToSetpoints(climbingSubsystem, 0, 0, 4, 1));

    //shooterSubsystem.setDefaultCommand(new IdleShooter(shooterSubsystem));

    /* INTAKE */

    // Run Intake-in while the left bumper is held
    new JoystickButton(joystick, LEFT_BUMPER).whenHeld(new DeployIntake(intakeSubsystem).andThen(new RunIntake(intakeSubsystem, loaderSubsystem))).whenReleased(new RetractIntake(intakeSubsystem).andThen(new RejectBall(loaderSubsystem, shooterSubsystem, true)));

    // Run reverse intake when right bumper is pressed
    new JoystickButton(joystick, RIGHT_BUMPER).whenHeld(new DeployIntake(intakeSubsystem).andThen(new EjectBalls(intakeSubsystem, loaderSubsystem))).whenReleased(new RetractIntake(intakeSubsystem));

    //loaderSubsystem.setDefaultCommand(new RejectBall(loaderSubsystem, shooterSubsystem, rejectBalls));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // driveSubsystem.resetOdometry(new Pose2d());
    // System.err.println(autoChooser.getSelected().getName());

    // Follow path, then cut voltage to motors (stop)

    if (autoChooser.getSelected() == null) {
      return new InstantCommand();
    }
    return autoChooser.getSelected().andThen(driveSubsystem::disable).andThen(shooterSubsystem::disable)
        .andThen(intakeSubsystem::disable).andThen(loaderSubsystem::disable);
  }

  public void testModeRunArms() {
    climbingSubsystem.setTestExtenderPercent(joystick.getLeftY() * 0.4, joystick.getRightY() * 0.4);
    climbingSubsystem.setTestRotatorPercent(-joystick.getLeftTriggerAxis() * 0.1,
        -joystick.getRightTriggerAxis() * 0.1);
  }

  public void setDriveMode() {
    driveMode = false;
  }

  public void setOverrideRejectBalls(boolean overrideRejectBalls) {
    // this.overrideRejectBalls = overrideRejectBalls;
  }

  public void clearClimberStickies() {
    climbingSubsystem.clearStickies();
  }

  public void testModeRunIntake() {
    double leftVolts = 0, rightVolts = 0;
    
    if (joystick.getXButton()) {
      leftVolts = -1.5;
    }
    if (joystick.getBButton()) {
      rightVolts = -1.5;
    }
    if(joystick.getYButton()) {
      intakeSubsystem.resetIntake();
    }
    intakeSubsystem.setDeployMotorsVolts(leftVolts, rightVolts);
  }

  public void setTeamColor() {
    isTeamRed = NetworkTablesHelper.getBoolean("FMSInfo", "IsRedAlliance", false);
  }

  public void updatePose() {
    driveSubsystem.updatePose();
  }

  /**
   * 
   * @return True if red, false if blue
   */
  public static boolean getTeamColor() {
    return isTeamRed;
  }

  public void logLimits() {
    SmartDashboard.putBoolean("back left limit", climbingSubsystem.isLeftBackAtLimit());
    SmartDashboard.putBoolean("back right limit", climbingSubsystem.isRightBackAtLimit());
    SmartDashboard.putBoolean("front left limit", climbingSubsystem.isLeftFrontAtLimit());
    SmartDashboard.putBoolean("front right limit", climbingSubsystem.isRightFrontAtLimit());
  }

}
