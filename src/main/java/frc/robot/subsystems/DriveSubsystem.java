// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class DriveSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonFX leftMotorTop = new WPI_TalonFX(DriveConstants.leftMotorTopPort);

  private WPI_TalonFX rightMotorTop = new WPI_TalonFX(DriveConstants.rightMotorTopPort);

  private MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(new MotorController[] { leftMotorTop,
      new WPI_TalonFX(DriveConstants.leftMotorFrontPort), new WPI_TalonFX(DriveConstants.leftMotorBackPort) });

  private MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(
      new MotorController[] { rightMotorTop, new WPI_TalonFX(DriveConstants.rightMotorFrontPort),
          new WPI_TalonFX(DriveConstants.rightMotorBackPort) });

  private DifferentialDrive m_Drive;

  private PIDController leftPID = new PIDController(DriveConstants.leftKP, 0, 0);
  private PIDController rightPID = new PIDController(DriveConstants.rightKP, 0, 0);

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv,
      DriveConstants.ka);

  private Pose2d pose = new Pose2d();

  private DifferentialDriveKinematics m_DriveKinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);

  private DifferentialDriveOdometry m_DriveOdometry = new DifferentialDriveOdometry(new Rotation2d(), pose);

  private AHRS ahrs = new AHRS();

  public DriveSubsystem() {
    m_Drive = new DifferentialDrive(rightMotorControllerGroup, leftMotorControllerGroup);


    ahrs.calibrate();
    ahrs.zeroYaw();

    leftMotorTop.configFactoryDefault();
    rightMotorTop.configFactoryDefault();

    

    rightMotorControllerGroup.setInverted(true);

    m_Drive.setMaxOutput(DriveConstants.maxDriveOutput);

  }

  /**
   * Drives the robot using arcade controls. Intend use for inline command
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

  /**
   * @return current heading from gyro in a {@link Rotation2d}
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-ahrs.getYaw());
  }

  /**
   * @return total distance right encoder has traveled in meters
   */

  public double getDistanceRight() {
    // Total clicks / clicks per rotation * gear ratio * wheel diameter * pi
    // Converts clicks to total rotation to distance travelled
    return -rightMotorTop.getSelectedSensorPosition(0) / DriveConstants.falcon500ClicksPerRot * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
        * Math.PI;
  }

  /**
   * 
   * @return total distance left encoder has traveled in meters
   */
  public double getDistanceLeft() {
    // Total clicks / clicks per rotation * gear ratio * wheel diameter * pi
    // Converts clicks to total rotation to distance travelled
    return leftMotorTop.getSelectedSensorPosition(0) / DriveConstants.falcon500ClicksPerRot * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
        * Math.PI;

  }

  /**
   * 
   * @return Wheel speeds from encoders (m/s)
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // Velocity in clicks per 100ms / clicks per rotation * gear ratio * wheel diameter * pi * 10
    // COnvert to motor ration per 100ms then convert to wheel rotation per 100ms then to meters per 100ms then to per sec

    return new DifferentialDriveWheelSpeeds(
        leftMotorTop.getSelectedSensorVelocity() / DriveConstants.falcon500ClicksPerRot * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
            * Math.PI * 10,
        -rightMotorTop.getSelectedSensorVelocity() / DriveConstants.falcon500ClicksPerRot * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
            * Math.PI * 10 );
  }

  @Override
  public void periodic() {
    pose = m_DriveOdometry.update(getHeading(), getDistanceLeft(), getDistanceRight());

    // Print info
    SmartDashboard.putNumber("Right speed (m/s)", getWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("Left speed (m/s)", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Right distance (m)", getDistanceRight());
    SmartDashboard.putNumber("Left distance (m)", getDistanceLeft());

    SmartDashboard.putNumber("x", pose.getX());
    SmartDashboard.putNumber("y", pose.getY());
    SmartDashboard.putNumber("yaw", pose.getRotation().getDegrees());

  }

  public void resetOdometry(Pose2d startingPose) {
    resetEncoders();
    m_DriveOdometry.resetPosition(startingPose, getHeading());
  
    ahrs.setAngleAdjustment(-90);
    ahrs.zeroYaw();
    ahrs.calibrate();
  }

  private void resetEncoders() {
    leftMotorTop.setSelectedSensorPosition(0);
    rightMotorTop.setSelectedSensorPosition(0);
  }

  // Getters
  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPid() {
    return leftPID;
  }

  public PIDController getRightPid() {
    return rightPID;
  }

  public DifferentialDriveKinematics getKinematics() {
    return m_DriveKinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

}