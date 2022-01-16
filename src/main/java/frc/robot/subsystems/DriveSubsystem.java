// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

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
  private MotorControllerGroup leftMotorControllerGroup;
  private MotorControllerGroup rightMotorControllerGroup;

  // Controllers
  private PIDController leftPID;
  private PIDController rightPID;
  private SimpleMotorFeedforward feedforward;

  // Pose & differential drive
  private Pose2d pose;
  private DifferentialDrive m_Drive;
  private DifferentialDriveOdometry m_DriveOdometry;

  // Gyro
  private AHRS ahrs;

  public DriveSubsystem() {
    constructorHelper();

    // Zero sensors
    ahrs.calibrate();
    resetOdometry(new Pose2d());
    leftMotorTop.configFactoryDefault();
    rightMotorTop.configFactoryDefault();

    // FIXME is it strange to invert a motor group and then have to report negative values in getDistanceRight()?
    rightMotorControllerGroup.setInverted(true);
    m_Drive.setMaxOutput(DriveConstants.maxDriveOutput);
  }

  /**
   * Drives the robot using arcade controls. Intend use for inline default command
   * 
   * @param fwd supplier for forward movement
   * @param rot supplier for rotation
   */
  public void arcadeDrive(DoubleSupplier fwd, DoubleSupplier rot) {
    m_Drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble());
  }

  /**
   * Drive the robot through tank drive using volts
   * 
   * @param leftVolts  left motor speeds in volts
   * @param rightVolts right motor speeds in volts
   */
  public void tankDriveVolts(Double leftVolts, Double rightVolts) {
    leftMotorControllerGroup.setVoltage(leftVolts);
    rightMotorControllerGroup.setVoltage(rightVolts);
  }

  @Override
  public void periodic() {
    pose = m_DriveOdometry.update(getHeading(), getDistanceLeft(), getDistanceRight());
    debugLog(DriveConstants.debug);
  }

  /**
   * Resets encoders to 0 and gyro yaw to 0
   * @param startingPose Pose to initialize odometry object to
   */
  public void resetOdometry(Pose2d startingPose) {
    resetEncoders();
    
    // TODO test if offset fixes auto routines
    ahrs.setAngleAdjustment(-90);
    ahrs.zeroYaw();
    
    // FIXME: The gyroscope angle does not need to be reset here on the user's robot code. The library automatically takes care of offsetting the gyro angle.
    m_DriveOdometry.resetPosition(startingPose, getHeading());
  }

  /**
   * Put any logging info here to not clutter up periodic.
   * @param enabled Print logging info?
   */
  private void debugLog(boolean enabled) {
    if(!enabled) return;
    // Speeds
    SmartDashboard.putNumber("Right speed (m/s)", getWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("Left speed (m/s)", getWheelSpeeds().leftMetersPerSecond);
    // Distance
    SmartDashboard.putNumber("Right distance (m)", getDistanceRight());
    SmartDashboard.putNumber("Left distance (m)", getDistanceLeft());
    // Position
    SmartDashboard.putNumber("x", pose.getX());
    SmartDashboard.putNumber("y", pose.getY());
    // Rotation
    SmartDashboard.putNumber("yaw", pose.getRotation().getDegrees());
  }

  // --- Getters ---

  static double getEncoderDistance(WPI_TalonFX targetMotor) {
    // Total clicks / clicks per rotation * gear ratio * wheel diameter * pi
    // Converts clicks to total rotation to distance travelled
    return targetMotor.getSelectedSensorPosition(0) / DriveConstants.falcon500ClicksPerRot * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter * Math.PI;
  }

  static double getEncoderVelocity(WPI_TalonFX targetMotor) {
    // Velocity in clicks per 100ms / clicks per rotation * gear ratio * wheel diameter * pi * 10
    // Convert to motor ration per 100ms then convert to wheel rotation per 100ms then to meters per 100ms then to per sec
    return targetMotor.getSelectedSensorVelocity() / DriveConstants.falcon500ClicksPerRot * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter * Math.PI * 10;
  }

  /**
   * @return current heading from gyro in a {@link Rotation2d}
   */
  public Rotation2d getHeading() {
    // TODO see if reversing gyro yaw fixes auto routine
    return Rotation2d.fromDegrees(-ahrs.getYaw());
  }

  /**
   * @return total distance right encoder has traveled in meters
   */
  public double getDistanceRight() {
    return -getEncoderDistance(rightMotorTop);
  }

  /**
   * @return total distance left encoder has traveled in meters
   */
  public double getDistanceLeft() {
    return getEncoderDistance(leftMotorTop);
  }

  /**
   * @return Wheel speeds from encoders (m/s)
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getEncoderVelocity(leftMotorTop), -getEncoderVelocity(rightMotorTop));
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

  private void resetEncoders() {
    leftMotorTop.setSelectedSensorPosition(0);
    rightMotorTop.setSelectedSensorPosition(0);
  }

  // --- Constructor helper ---

  /**
   * Moves all the ugly instantiation out of the way.
   */
  private void constructorHelper() {
    leftMotorTop = new WPI_TalonFX(DriveConstants.leftMotorTopPort);
    rightMotorTop = new WPI_TalonFX(DriveConstants.rightMotorTopPort);

    leftMotorControllerGroup = new MotorControllerGroup(new MotorController[] { leftMotorTop,
        new WPI_TalonFX(DriveConstants.leftMotorFrontPort), new WPI_TalonFX(DriveConstants.leftMotorBackPort) });

    rightMotorControllerGroup = new MotorControllerGroup( new MotorController[] { rightMotorTop,
        new WPI_TalonFX(DriveConstants.rightMotorFrontPort), new WPI_TalonFX(DriveConstants.rightMotorBackPort) });

    m_Drive = new DifferentialDrive(rightMotorControllerGroup, leftMotorControllerGroup);

    leftPID = new PIDController(DriveConstants.leftKP, 0, 0);
    rightPID = new PIDController(DriveConstants.rightKP, 0, 0);

    feedforward = new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka);

    // TODO getting error on startup from differential drive not updated often enough. Need to set this to true value?
    pose = new Pose2d();

    m_DriveOdometry = new DifferentialDriveOdometry(new Rotation2d(), pose);

    ahrs = new AHRS();
  }
}
