// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class DriveSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Motors
  private WPI_TalonFX leftMotorTop;
  private WPI_TalonFX rightMotorTop;
  private WPI_TalonFX leftMotorFront;
  private WPI_TalonFX rightMotorBack;
  private WPI_TalonFX leftMotorBack;
  private WPI_TalonFX rightMotorFront;
  private MotorControllerGroup leftMotorControllerGroup;
  private MotorControllerGroup rightMotorControllerGroup;

  // Controllers
  private PIDController leftPID;
  private PIDController rightPID;
  private SimpleMotorFeedforward feedforward;

  // Pose & differential drive
  private Pose2d pose;
  private DifferentialDrive drive;
  private DifferentialDriveOdometry driveOdometry;
  private SlewRateLimiter decelFilter = new SlewRateLimiter(DECELERATION_SLEW_RATE_LIMITER);
  private SlewRateLimiter accelFilter = new SlewRateLimiter(ACCELERATION_SLEW_RATE_LIMITER);
  private double previousPercentage = 0;

  // Gyro
  private AHRS gyro;

  // log
  ShuffleboardTab tab;
  Field2d field2d = new Field2d();

  public DriveSubsystem() {
    constructorHelper();
    // Zero sensors
    resetOdometry(new Pose2d());
    //leftMotorTop.configFactoryDefault();
    //rightMotorTop.configFactoryDefault();

    rightMotorControllerGroup.setInverted(RIGHT_INVERTED);
    leftMotorControllerGroup.setInverted(LEFT_INVERTED);
    drive.setMaxOutput(MAX_DRIVE_OUTPUT_PERCENT);
    drive.arcadeDrive(0, 0);
    

    tab = Shuffleboard.getTab(this.getName());

    gyro.reset();
    

    logData();

    LiveWindow.disableAllTelemetry();
  }

  @Override
  public void periodic() {
    pose = driveOdometry.update(getHeading(), getDistanceLeft(), getDistanceRight());
    field2d.setRobotPose(pose);
  }

  /**
   * Drives the robot using arcade controls. Intend use for inline default
   * command.
   * Slew filtering will be applied to the raw inputs
   * 
   * @param fwd supplier for forward movement
   * @param rot supplier for rotation
   */
  public void arcadeDrive(DoubleSupplier fwd, DoubleSupplier rot){
    double currentPercentage = fwd.getAsDouble();
    double slewOutput;
    if(Math.abs(currentPercentage) > previousPercentage) { // Speeding up, use acceleration slew limier
      slewOutput = accelFilter.calculate(currentPercentage);
      decelFilter.calculate(currentPercentage);
    } else { // Slowing down, use deceleration slew limiter
      slewOutput = decelFilter.calculate(currentPercentage);
      accelFilter.calculate(currentPercentage);
    }
    drive.arcadeDrive(slewOutput, rot.getAsDouble());
    previousPercentage = Math.abs(currentPercentage);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotorControllerGroup.setVoltage(leftVolts);
    rightMotorControllerGroup.setVoltage(rightVolts);
    drive.feed();
  }

  public void logData() {
   
    tab.addNumber("angle", () -> pose.getRotation().getDegrees());
    tab.addNumber("heading", () -> getHeading().getDegrees());
    tab.add(this);
    tab.add(leftMotorTop);
    tab.add(rightMotorTop);
    tab.add(field2d);
    tab.add(leftPID);
    tab.add(rightPID);
    tab.addNumber("x", () -> pose.getX());
    tab.addNumber("y",  () -> pose.getY());
    tab.addNumber("Left Speed",  () -> getWheelSpeeds().leftMetersPerSecond);
    tab.addNumber("Right Speed",  () -> (getWheelSpeeds().rightMetersPerSecond));
   
  }

  // --- Getters ---

  private static double getEncoderDistance(WPI_TalonFX targetMotor) {

    // Converts clicks to distance in meters
    return targetMotor.getSelectedSensorPosition(0) * CLICKS_TO_METERS;
  }

  private static double getEncoderVelocity(WPI_TalonFX targetMotor) {
    // Convert from clicks per 100ms to meters per 100ms then to per sec
    return targetMotor.getSelectedSensorVelocity() * CLICKS_TO_METERS * 10;
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-Math.IEEEremainder(gyro.getAngle(), 360));
  }

  public double getDistanceRight() {
    return (RIGHT_INVERTED ? -1 : 1) * getEncoderDistance(rightMotorTop);
  }

  public double getDistanceLeft() {
    return (LEFT_INVERTED ? -1 : 1) * getEncoderDistance(leftMotorTop);
  }

  public double getVelocityRight() {
    return (RIGHT_INVERTED ? -1 : 1) * getEncoderVelocity(rightMotorTop);
  }

  public double getVelocityLeft() {
    return (LEFT_INVERTED ? -1 : 1) * getEncoderVelocity(leftMotorTop);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getVelocityLeft(), getVelocityRight());
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPid() {
    return leftPID;
  }

  public PIDController getRightPid() {
    return rightPID;
  }

  public Pose2d getPose() {
    return pose;

  }

  // --- Setters ---


  public void disable(){
    tankDriveVolts(0, 0);
  }

  private void resetEncoders() {
    leftMotorTop.setSelectedSensorPosition(0);
    rightMotorTop.setSelectedSensorPosition(0);
  }

  /**
   * Resets encoders to 0
   * 
   * @param startingPose Pose to initialize odometry object to
   */
  public void resetOdometry(Pose2d startingPose) {
    resetEncoders();
    gyro.reset();
    driveOdometry.resetPosition(startingPose, getHeading());
  }

  // --- Constructor helper ---

  /**
   * Moves all the ugly instantiation out of the way.
   */
  private void constructorHelper() {
    leftMotorTop = new WPI_TalonFX(LEFT_MOTOR_TOP_PORT, Constants.CANIVORE_NAME);
    rightMotorTop = new WPI_TalonFX(RIGHT_MOTOR_TOP_PORT, Constants.CANIVORE_NAME);
    leftMotorFront = new WPI_TalonFX(LEFT_MOTOR_FRONT_PORT, Constants.CANIVORE_NAME);
    rightMotorFront = new WPI_TalonFX(RIGHT_MOTOR_FRONT_PORT, Constants.CANIVORE_NAME);
    leftMotorBack = new WPI_TalonFX(LEFT_MOTOR_BACK_PORT, Constants.CANIVORE_NAME);
    rightMotorBack = new WPI_TalonFX(RIGHT_MOTOR_BACK_PORT, Constants.CANIVORE_NAME);

    leftMotorControllerGroup = new MotorControllerGroup(new MotorController[] { leftMotorTop,
        updateGeneralStatusFrame(leftMotorFront), updateGeneralStatusFrame(leftMotorBack) });

    rightMotorControllerGroup = new MotorControllerGroup(new MotorController[] { rightMotorTop,
        updateGeneralStatusFrame(rightMotorFront), updateGeneralStatusFrame(rightMotorBack)});

    drive = new DifferentialDrive(rightMotorControllerGroup, leftMotorControllerGroup);

    leftPID = new PIDController(LEFT_KP, 0, 0);
    rightPID = new PIDController(RIGHT_KP, 0, 0);

    feedforward = new SimpleMotorFeedforward(KS, KV, KA);

    pose = new Pose2d();

    driveOdometry = new DifferentialDriveOdometry(new Rotation2d(), pose);

    gyro = new AHRS();

    leftMotorTop.clearStickyFaults();
    rightMotorTop.clearStickyFaults();
    leftMotorFront.clearStickyFaults();
    rightMotorFront.clearStickyFaults();
    leftMotorBack.clearStickyFaults();
    rightMotorBack.clearStickyFaults();
  }

  private WPI_TalonFX updateGeneralStatusFrame(WPI_TalonFX motor) {
    motor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    return motor;
  }

  public void resetPose2D(Pose2d pose) {
    rightMotorTop.setSelectedSensorPosition(0);
    leftMotorTop.setSelectedSensorPosition(0);
    gyro.reset();
    gyro.setAngleAdjustment(pose.getRotation().getDegrees());
    driveOdometry.resetPosition(pose, getHeading());
  }

}
