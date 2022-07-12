// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix.led.CANdle;

import frc.robot.Constants.CandleConstants;

/** Add your docs here. */
public class Candle {
    private static Candle CANDLE = null;

    private boolean clearAllAnims;

    private CANdle candle = new CANdle(CandleConstants.CANDLE_ID);
    public enum LEDState {
      Fire,
      Idle,
      Intake,
      Climbing,
      Auto,
      ReverseIntake,
      Disabled,
      Off
    }
    private LEDState currentState = LEDState.Off;


    private DoubleSupplier fwd;
    private DoubleSupplier rot;


    public Candle() {
        changeLedState(LEDState.Off);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
      
        candle.configAllSettings(configAll, 100);
        candle.configLOSBehavior(true);
    }
    
    public void incrementAnimation() {
      System.out.println(currentState);
      switch(currentState) {
          case Off: changeLedState(LEDState.Fire); break;
          case Fire: changeLedState(LEDState.Intake); break;
          case Intake: changeLedState(LEDState.Idle); break;
          case Idle: changeLedState(LEDState.Climbing); break;
          case Climbing: changeLedState(LEDState.Fire); break;
          case ReverseIntake: changeLedState(LEDState.ReverseIntake); break;
      }
    }
    
    public void changeLedState(LEDState fire) {
    
        for(int i = 0; i < 10; ++i) {
          candle.clearAnimation(i);
      }
        switch (fire) {
          case Fire: 
            candle.animate(new FireAnimation(1, 0.7, 32, 0.8, 0.4, false, 8), 1);
            candle.animate(new FireAnimation(1, 0.7, 32, 0.8, 0.4, true, 120 +8 -32 ), 2);
            candle.animate(new FireAnimation(1, 0.7, 13, 0.8, 0.6, false, 68 ), 3);
            candle.animate(new FireAnimation(1, 0.7, 14, 0.8, 0.3, true, 68 - 16), 4);
            currentState = LEDState.Fire;
            //candle.animate(new TwinkleAnimation(255, 18, 213, 0, .1, 120 + 8 - 58, TwinklePercent.Percent30, 58), 3);
            break;
          case Idle: 
          
            currentState = LEDState.Idle;
    
            break;
          case Intake: 
            candle.animate(new ColorFlowAnimation(128, 20, 60, 0, 0.9, 60, Direction.Forward,  8), 1);
            candle.animate(new ColorFlowAnimation(128, 20, 60, 0, 0.9, 60, Direction.Backward,  68), 2);
    
            currentState = LEDState.Intake;
            
          
            
    
            break;
          case Climbing: 
            candle.animate(new LarsonAnimation(255, 140, 0, 0, 0.7, 120, BounceMode.Back, 3, 8), 1);
            currentState = LEDState.Climbing;
    
    
            break;
          case Auto: 

          candle.animate(new RainbowAnimation(1, 1, 120, false, 8));
          currentState = LEDState.Auto;
            break;
          case ReverseIntake: 
    
          candle.animate(new ColorFlowAnimation(255, 20, 0, 0, 0.9, 60, Direction.Backward,  8), 1);
            candle.animate(new ColorFlowAnimation(255, 20, 0, 0, 0.9, 60, Direction.Forward,  68), 2);

          currentState = LEDState.ReverseIntake;
            break;
          case Disabled:

          candle.animate(new TwinkleAnimation(255, 255, 255, 0, 0.4, 120, TwinklePercent.Percent30, 8), 1);     
          
          currentState = LEDState.Disabled;
            break;
          case Off:
          
          currentState = LEDState.Off;
            break;
          default:
            break;
        }
      }
    
      public void periodic() {

        if(currentState == LEDState.Idle) {

            double leftSpeed = MathUtil.clamp(fwd.getAsDouble() + rot.getAsDouble(), -1, 1);
            double rightSpeed = MathUtil.clamp(fwd.getAsDouble() - rot.getAsDouble(), -1, 1);


            if(rightSpeed > 0) {
            candle.setLEDs(0, 255, 0, 255, 8, (int)(60 * rightSpeed) + 5);
            candle.setLEDs(0, 0, 0, 0, 8 +(int)(60 * rightSpeed) + 5, 60 - (int)(60 * rightSpeed) );
            }
            if(leftSpeed > 0) {
            candle.setLEDs(255, 0, 0, 255, 128 - (int)(60 * leftSpeed), (int)(60 * leftSpeed));
            candle.setLEDs(0, 0, 0, 0, 68, 60 - (int)(60 * leftSpeed));
            }
            


            // if(rightSpeed < 0) {
            //     candle.setLEDs(255, 0, 0, 255, 8, (int)(60 * rightSpeed));
            //     candle.setLEDs(0, 0, 0, 0, (int)(60 * rightSpeed), 60 - (int)(60 * rightSpeed) );
            // }
            // if(leftSpeed < 0) {
            //     candle.setLEDs(255, 0, 0, 255, 68 - (int)(60 * leftSpeed), (int)(60 * leftSpeed));
            //     candle.setLEDs(0, 0, 0, 0, 68, 60 - (int)(60 * leftSpeed));
            //  }
        }


      }

      public void setRobotSpeed(DoubleSupplier fwd, DoubleSupplier rot) {

        this.fwd = fwd;
        this.rot = rot;
      }
    
      public double getVbat() { return candle.getBusVoltage(); }
      public double get5V() { return candle.get5VRailVoltage(); }
      public double getCurrent() { return candle.getCurrent(); }
      public double getTemperature() { return candle.getTemperature(); }
      public void configBrightness(double percent) { candle.configBrightnessScalar(percent, 0); }
      public void configLos(boolean disableWhenLos) { candle.configLOSBehavior(disableWhenLos, 0); }
      public void configLedType(LEDStripType type) {  candle.configLEDType(type, 0); }
      public void configStatusLedBehavior(boolean offWhenActive) { candle.configStatusLedState(offWhenActive, 0); }
    
    
      
      public void clearAllAnims() {clearAllAnims = true;}
    
     
      public void setAllToColor(int r, int g, int b) {
    
        candle.setLEDs(r, g, b, 255, 8, 50);
    
    
      }


    /**
     * @return The Single instance of Singleton LimeLight
     */
    public static Candle getInstance() {
        // To ensure only one instance is created
        if (CANDLE == null) {
            CANDLE = new Candle();
        }
        return CANDLE;
    }

}
