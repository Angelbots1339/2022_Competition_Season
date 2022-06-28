// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CandleConstants;

public class CandleSubsystem extends SubsystemBase {

  private CANdle candle = new CANdle(CandleConstants.CANDLE_ID);



  /** Creates a new CandleSubsystem. */
  public CandleSubsystem() {


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



  public void setAllToColor(int r, int g, int b) {

    candle.setLEDs(r, g, b, 255, 8, 50);


  }
}
