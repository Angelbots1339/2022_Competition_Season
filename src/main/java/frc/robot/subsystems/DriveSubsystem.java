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
import com.kauailabs.navx.frc.AHRS;



import java.util.function.DoubleSupplier;


/** Add your docs here. */
public class DriveSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax leftMotorTop = new CANSparkMax(DriveConstants.leftMotorTopPort, MotorType.kBrushless);
  private CANSparkMax leftMotorFront = new CANSparkMax(DriveConstants.leftMotorFrontPort, MotorType.kBrushless);
  private CANSparkMax leftMotorBack = new CANSparkMax(DriveConstants.leftMotorBackPort, MotorType.kBrushless);
  private CANSparkMax rightMotorTop = new CANSparkMax(DriveConstants.rightMotorTopPort, MotorType.kBrushless);
  private CANSparkMax rightMotorFront = new CANSparkMax(DriveConstants.rightMotorFrontPort, MotorType.kBrushless);
  private CANSparkMax rightMotorBack = new CANSparkMax(DriveConstants.rightMotorBackPort, MotorType.kBrushless);

  private MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(new MotorController[]{leftMotorBack, leftMotorFront, leftMotorTop});
  private MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(new MotorController[]{rightMotorBack, rightMotorFront, rightMotorTop});

  private DifferentialDrive m_Drive = new DifferentialDrive(rightMotorControllerGroup, leftMotorControllerGroup);

  private PIDController leftPID = new PIDController(DriveConstants.leftKP, 0, 0);
  private PIDController rightPID = new PIDController(DriveConstants.rightKP, 0, 0);

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka);

  private Pose2d pose = new Pose2d();

  private DifferentialDriveKinematics m_DriveKinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);
                                                                                                                     
  private DifferentialDriveOdometry m_DriveOdometry = new DifferentialDriveOdometry(new Rotation2d(), pose);

  private AHRS ahrs = new AHRS();

  public DriveSubsystem() {
    ahrs.calibrate();
      
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
    return null;//Rotation2d.fromDegrees(-gyro.getAngle());
  }

  /**
   * @return total distance right encoder has traveled in meters
   */

  public double getDistanceRight() {
    // todo add clicks per rot and wheel r and gear ratio
    return rightMotorTop.getEncoder().getPosition() * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
        * Math.PI;
  }

  /**
   * 
   * @return total distance left encoder has traveled in meters
   */
  public double getDistanceLeft() {
    // todo add clicks per rot and wheel r and gear ratio
    return leftMotorTop.getEncoder().getPosition() * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
        * Math.PI;

  }
  /**
   * 
   * @return Wheel speeds in type {@link DifferentialDriveWheelSpeeds} from encoders in meters per sec
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMotorTop.getEncoder().getVelocity() * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
            * Math.PI / 60,
        rightMotorTop.getEncoder().getVelocity() * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
            * Math.PI / 60);
  }

 @Override
  public void periodic() {
    m_Drive.setMaxOutput(DriveConstants.maxDriveOutput);
    
    
  //  pose = m_DriveOdometry.update(getHeading(), getDistanceRight(), getDistanceLeft());
    SmartDashboard.putNumber("Current Draw LF", leftMotorFront.getOutputCurrent());
    SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());
    /* Display tilt-corrected, Magnetometer-based heading (requires             */
    /* magnetometer calibration to be useful)                                   */
    SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());
    /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
    SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());
    /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
    SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
    SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());
    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
    SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
    SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());
    /* Display estimates of velocity/displacement.  Note that these values are  */
    /* not expected to be accurate enough for estimating robot position on a    */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially      */
    /* double (displacement) integration.                                       */
    SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
    SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
    SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());
    /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
    /* NOTE:  These values are not normally necessary, but are made available   */
    /* for advanced users.  Before using this data, please consider whether     */
    /* the processed data (see above) will suit your needs.                     */
    SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
    SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
    SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
    SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
    SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
    SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
    SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
    SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
    /* Omnimount Yaw Axis Information                                           */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
    AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());
    /* Sensor Board Information                                                 */
    SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());
    /* Quaternion Data                                                          */
    /* Quaternions are fascinating, and are the most compact representation of  */
    /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
    /* from the Quaternions.  If interested in motion processing, knowledge of  */
    /* Quaternions is highly recommended.                                       */
    SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
    SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
    SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
    SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());
    /* Sensor Data Timestamp */
    SmartDashboard.putNumber("SensorTimestamp", ahrs.getLastSensorTimestamp());
    /* Connectivity Debugging Support                                           */
    SmartDashboard.putNumber("IMU_Byte_Count", ahrs.getByteCount());
    SmartDashboard.putNumber("IMU_Update_Count", ahrs.getUpdateCount());
  }

  //Getters
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
