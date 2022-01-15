// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
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
import edu.wpi.first.wpilibj.util.Color;
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

  private DifferentialDrive m_Drive = new DifferentialDrive(rightMotorControllerGroup, leftMotorControllerGroup);

  private PIDController leftPID = new PIDController(DriveConstants.leftKP, 0, 0);
  private PIDController rightPID = new PIDController(DriveConstants.rightKP, 0, 0);

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv,
      DriveConstants.ka);

  private Pose2d pose = new Pose2d();

  private DifferentialDriveKinematics m_DriveKinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);

  private DifferentialDriveOdometry m_DriveOdometry = new DifferentialDriveOdometry(new Rotation2d(), pose);

  private AHRS ahrs = new AHRS();

  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private ColorSensorV3 mColorSensorV3 = new ColorSensorV3(i2cPort);

  public DriveSubsystem() {
    ahrs.calibrate();
    ahrs.zeroYaw();

    leftMotorTop.configFactoryDefault();
    rightMotorTop.configFactoryDefault();

    rightMotorControllerGroup.setInverted(true);


  }

  /**
   * Drives the robot using arcade controls. Intend use for inline command reason
   * for Suppliers
   * 
   * @param fwd supplier for forward movement
   * @param rot supplier for rotation rotation
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
    return Rotation2d.fromDegrees(ahrs.getYaw());
  }

  /**
   * @return total distance right encoder has traveled in meters
   */

  public double getDistanceRight() {
    // Total clicks / clicks per rotation * gear ratio * wheel diameter * pi
    // Converts clicks to total rotation to distance travelled
    return rightMotorTop.getSelectedSensorPosition(0) / DriveConstants.falcon500ClicksPerRot * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
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
   * @return Wheel speeds in type {@link DifferentialDriveWheelSpeeds} from
   *         encoders in meters per sec
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // Velocity in clicks per 100ms / clicks per rotation * gear ratio * wheel diameter * pi * 10
    // COnvert to motor ration per 100ms then convert to wheel rotation per 100ms then to meters per 100ms then to per sec

    return new DifferentialDriveWheelSpeeds(
        leftMotorTop.getSelectedSensorVelocity() / DriveConstants.falcon500ClicksPerRot * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
            * Math.PI * 10,
        rightMotorTop.getSelectedSensorVelocity() / DriveConstants.falcon500ClicksPerRot * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
            * Math.PI * 10 );
  }

  @Override
  public void periodic() {
    m_Drive.setMaxOutput(DriveConstants.maxDriveOutput);
    pose = m_DriveOdometry.update(getHeading(), getDistanceRight(), getDistanceLeft());

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
