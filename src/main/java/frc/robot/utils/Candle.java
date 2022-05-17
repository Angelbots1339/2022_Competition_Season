// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import frc.robot.Constants.CandleConstants;

/** Add your docs here. */
public class Candle {


    private CANdle candle = new CANdle(CandleConstants.CANDLE_ID);


    /** Creates a new CandleSubsystem. */
  public Candle() {

    candle.configLEDType(LEDStripType.GRB);
    candle.configStatusLedState(true);
    candle.configLOSBehavior(true);
    candle.configBrightnessScalar(CandleConstants.CANDLE_BRIGHTNESS);
    candle.clearStickyFaults();
    // candle.setLEDs(255, 0, 0, 255, 8, 10);

  }


  public void setToLarsonAnimation(int R, int G, int B) {


    candle.animate(new LarsonAnimation(R, G, B));
    
    
      }
    
      public void setToLarsonAnimation(int R, int G, int B, int W, double Speed, int numLed, BounceMode bounceMode, int size) {
    
        candle.animate(new LarsonAnimation(R, G, B, W, Speed, numLed, bounceMode, size));
    
    
    
      }
    
      public void setToIdleAnimation(){
    
        // candle.animate(new LarsonAnimation(CandleConstants.DEFAULT_R, CandleConstants.DEFAULT_G, CandleConstants.DEFAULT_B, CandleConstants.DEFAULT_W, CandleConstants.DEFAULT_SPEED, CandleConstants.DEFAULT_NUM_LED, CandleConstants.DEFAULT_BOUNCE_MODE, CandleConstants.DEFAULT_SIZE));
        // candle.animate(new ColorFlowAnimation(255, 0, 0, 255, .1, 60 * 2, Direction.Forward, 8), 1);
        // candle.setLEDs(255,0,0);
        candle.clearAnimation(1);
        candle.animate(new TwinkleAnimation(255, 0, 0, 0, .1, 60* 2, TwinklePercent.Percent100, 8), 1);
        // candle.setLEDs(255, 0, 0, 255, 8, 10);

      }

      public void setToShootingAnimation(){
        candle.clearAnimation(1);
        candle.animate(new TwinkleAnimation(0, 255, 0, 0, .1, 60* 2, TwinklePercent.Percent100, 8), 1);

        // candle.setLEDs(0, 255, 0, 255, 8, 10);
        // candle.setLEDs(0,0,0);
        // candle.animate(new ColorFlowAnimation(0, 255, 0, 255, .1, 60 * 2, Direction.Forward, 8), 1);



      }

    //   public void setToClimbingAnimation(){

    //     candle.setLEDs(255, 0, 0);

    //   }

    //   public void setToIntakeAnimation(){

    //     candle.setLEDs(0, 0, 255);

    //   }
    
    //   public void setAllToColor(int R, int G, int B) {
    
    //     candle.setLEDs(R, G, B);
    
    
    //   }
    



}
