// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LoaderConstants.*;


public class LoaderSubsystem extends SubsystemBase {
  
  private WPI_TalonFX loaderMotor = new WPI_TalonFX(LOADER_PORT);
  /** Creates a new LoaderSubsystem. */
  public LoaderSubsystem() {
    loaderMotor.configFactoryDefault();
    loaderMotor.setInverted(LOADER_INVERSE);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  /**
   * This will run the indexer wheel in the shooter
   * 
   * @param speed
   */
  public void runLoader(double speed) {
    loaderMotor.set(speed);
  }

  public void disable() {
    loaderMotor.set(0);
  }
}
