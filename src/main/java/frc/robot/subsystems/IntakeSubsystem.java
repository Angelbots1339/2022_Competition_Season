// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

  private WPI_TalonFX intakeMotor = new WPI_TalonFX(INTAKE_MOTOR_PORT);
  private WPI_TalonFX indexerLeftMotor = new WPI_TalonFX(INDEXER_LEFT_PORT);
  private WPI_TalonFX indexerRightMotor = new WPI_TalonFX(INDEXER_RIGHT_PORT);
  private WPI_TalonFX indexerUpperMotor = new WPI_TalonFX(INDEXER_UPPER_PORT);
  private Servo servoRight = new Servo(SERVO_RIGHT_PORT);
  private Servo servoLeft = new Servo(SERVO_LEFT_PORT);

  private ColorSensorV3 colorSensorHigh;
  private ColorSensorV3 colorSensorLow;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    /* TODO add color sensor initialization */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void deployIntake() {
    servoRight.setAngle(90);
    servoLeft.setAngle(90);
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void runIndexerHigh(double speed) {
    indexerUpperMotor.set(speed);
  }

  public void runIndexerLow(double speed) {
    indexerLeftMotor.set((INDEXER_LEFT_INVERSE ? -1 : 1) * speed);
    indexerRightMotor.set((INDEXER_RIGHT_INVERSE ? -1 : 1) * speed);
  }

  public boolean isBallHigh() {
    return colorSensorHigh.getProximity() > COLOR_SENSOR_PROXIMITY_THRESHOLD;
  }

  public boolean isBallLow() {
    return colorSensorHigh.getProximity() > COLOR_SENSOR_PROXIMITY_THRESHOLD;
  }

}