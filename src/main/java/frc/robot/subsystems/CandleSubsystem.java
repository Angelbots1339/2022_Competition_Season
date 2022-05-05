// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CandleConstants;
import frc.robot.utils.Logging;

public class CandleSubsystem extends SubsystemBase {

  private CANdle candle = new CANdle(CandleConstants.CANDLE_ID);

  public static ShuffleboardTab tab = Shuffleboard.getTab("CandleSubsystem");

  /** Creates a new CandleSubsystem. */
  public CandleSubsystem() {

    candle.configLEDType(LEDStripType.GRB);

    if(Logging.candle) {
      log();
    }
  }

  public void log() {
    tab.addNumber("Candle Current", () -> candle.getCurrent());
    tab.addNumber("Candle Temp", () -> candle.getTemperature());
  }


 @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setToLarsonAnimation(int R, int G, int B) {


candle.animate(new LarsonAnimation(R, G, B));


  }

  public void setToLarsonAnimation(int R, int G, int B, int W, double Speed, int numLed, BounceMode bounceMode, int size) {

    candle.animate(new LarsonAnimation(R, G, B, W, Speed, numLed, bounceMode, size));



  }

  public void setToDefaultAnimation(){

    candle.animate(new LarsonAnimation(CandleConstants.DEFAULT_R, CandleConstants.DEFAULT_G, CandleConstants.DEFAULT_B, CandleConstants.DEFAULT_W, CandleConstants.DEFAULT_SPEED, CandleConstants.DEFAULT_NUM_LED, CandleConstants.DEFAULT_BOUNCE_MODE, CandleConstants.DEFAULT_SIZE));

  }

  public void setAllToColor(int R, int G, int B) {

    candle.setLEDs(R, G, B);


  }

 
}
