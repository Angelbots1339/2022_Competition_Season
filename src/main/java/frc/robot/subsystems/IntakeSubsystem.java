// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ColorMUXed;

import static frc.robot.Constants.IntakeConstants.*;


public class IntakeSubsystem extends SubsystemBase {

  private WPI_TalonFX intakeMotor = new WPI_TalonFX(INTAKE_MOTOR_PORT);
  private WPI_TalonFX indexerLeftMotor = new WPI_TalonFX(INDEXER_LEFT_PORT);
  private WPI_TalonFX indexerRightMotor = new WPI_TalonFX(INDEXER_RIGHT_PORT);
  

  private ShuffleboardTab tab = Shuffleboard.getTab("Intake Subsystem");

  // TODO Warning: Set NavX voltage jumper to 3.3v
  //private ColorMUXed colorSensorHigh = new ColorMUXed(COLOR_SENSOR_HIGH_PORT);
  //private ColorMUXed colorSensorLow = new ColorMUXed(COLOR_SENSOR_LOW_PORT);
  private ColorSensorV3 colorSensorLow = new ColorSensorV3(Port.kMXP);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  
    intakeMotor.configFactoryDefault();
    indexerLeftMotor.configFactoryDefault();
    indexerRightMotor.configFactoryDefault();
    
    tab.addNumber("supply current", () -> intakeMotor.getSupplyCurrent());
    tab.addNumber("stator current", () -> intakeMotor.getStatorCurrent());
    tab.addNumber("ColorSensor low", () -> colorSensorLow.getProximity());

    indexerLeftMotor.setInverted(INDEXER_LEFT_INVERTED);
    indexerRightMotor.setInverted(INDEXER_RIGHT_INVERTED);
    intakeMotor.setInverted(INTAKE_INVERSE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
   * This will run the belts on the lower indexer
   * 
   * @param speed
   */
  public void runIndexerLow(double speed) {
    indexerLeftMotor.set(speed);
    indexerRightMotor.set(speed);
  }

  // public boolean isBallHigh() {
  //   return colorSensorHigh.getProximity() > COLOR_SENSOR_PROXIMITY_THRESHOLD;
  // }

  public boolean isBallLow() {
    return colorSensorLow.getProximity() > COLOR_SENSOR_PROXIMITY_THRESHOLD;
  }

  /**
   * Disables all motors 
   */
  public void Disable() {
    indexerLeftMotor.set(0);
    indexerRightMotor.set(0);
    intakeMotor.set(0);
  }

}