// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ColorMUXed;

import static frc.robot.Constants.IntakeConstants.*;
import edu.wpi.first.wpilibj.I2C;

public class IntakeSubsystem extends SubsystemBase {

  private WPI_TalonFX intakeMotor = new WPI_TalonFX(INTAKE_MOTOR_PORT);
  private WPI_TalonFX indexerLeftMotor = new WPI_TalonFX(INDEXER_LEFT_PORT);
  private WPI_TalonFX indexerRightMotor = new WPI_TalonFX(INDEXER_RIGHT_PORT);
  private WPI_TalonFX indexerUpperMotor = new WPI_TalonFX(INDEXER_UPPER_PORT);
  private Servo servoRight = new Servo(SERVO_RIGHT_PORT);
  private Servo servoLeft = new Servo(SERVO_LEFT_PORT); 

  // TODO Warning: Set NavX voltage jumper to 3.3v
  private ColorMUXed colorSensorHigh = new ColorMUXed(COLOR_SENSOR_HIGH_PORT);
  private ColorMUXed colorSensorLow = new ColorMUXed(COLOR_SENSOR_LOW_PORT);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  
    intakeMotor.configFactoryDefault();
    indexerLeftMotor.configFactoryDefault();
    indexerRightMotor.configFactoryDefault();
    indexerUpperMotor.configFactoryDefault();

    indexerLeftMotor.setInverted(INDEXER_LEFT_INVERSE);
    indexerRightMotor.setInverted(INDEXER_RIGHT_INVERSE);
    indexerUpperMotor.setInverted(INDEXER_UPPER_INVERSE);
    intakeMotor.setInverted(INTAKE_INVERSE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void deployIntake() {
    servoRight.setAngle(90);
    servoLeft.setAngle(90);
  }

  /**
   * This will spin the intake motor at the given speed
   * 
   * @param speed
   */
  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * This will run the indexer wheel in the shooter
   * 
   * @param speed
   */
  public void runIndexerHigh(double speed) {
    indexerUpperMotor.set(speed);
  }

  /**
   * This will run the belts on the lower indexer
   * 
   * @param speed
   */
  public void runIndexerLow(double speed) {
    indexerLeftMotor.set((INDEXER_LEFT_INVERSE ? -1 : 1) * speed);
    indexerRightMotor.set((INDEXER_RIGHT_INVERSE ? -1 : 1) * speed);
  }

  public boolean isBallHigh() {
    return colorSensorHigh.getProximity() > COLOR_SENSOR_PROXIMITY_THRESHOLD;
  }

  public boolean isBallLow() {
    return colorSensorLow.getProximity() > COLOR_SENSOR_PROXIMITY_THRESHOLD;
  }

}