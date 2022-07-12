// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CandleConstants;

public class CandleSubsystem extends SubsystemBase {


  private boolean clearAllAnims;

  private CANdle candle = new CANdle(CandleConstants.CANDLE_ID);
  public enum LEDState {
    Fire,
    Shoot, 
    Idle,
    Intake,
    Climbing,
    Auto,
    Off
  }
  private LEDState currentState = LEDState.Off;

  public CandleSubsystem() {
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
  }
}

  public void changeLedState(LEDState state) {

    for(int i = 0; i < 10; ++i) {
      candle.clearAnimation(i);
  }
    switch (state) {
      case Fire: 
        candle.animate(new FireAnimation(1, 0.7, 32, 0.8, 0.4, false, 8), 1);
        candle.animate(new FireAnimation(1, 0.7, 32, 0.8, 0.4, true, 120 +8 -32 ), 2);
        candle.animate(new FireAnimation(1, 0.7, 13, 0.8, 0.6, false, 68 ), 3);
        candle.animate(new FireAnimation(1, 0.7, 14, 0.8, 0.3, true, 68 - 16), 4);
        currentState = LEDState.Fire;
        //candle.animate(new TwinkleAnimation(255, 18, 213, 0, .1, 120 + 8 - 58, TwinklePercent.Percent30, 58), 3);
        break;
      case Shoot: 
       currentState = LEDState.Shoot;

        break;
      case Idle: 
        candle.animate(new TwinkleAnimation(255, 255, 255, 0, 0.4, 120, TwinklePercent.Percent30, 8), 1);            
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

      currentState = LEDState.Auto;
        break;
      case Off:
      
      currentState = LEDState.Off;
        break;
      default:
        break;
    }
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

  @Override
  public void periodic() {
  
    // This method will be called once per scheduler run

    if(clearAllAnims) {
      clearAllAnims = false;
      for(int i = 0; i < 10; ++i) {
          candle.clearAnimation(i);
      }
  }

  if(currentState == LEDState.Climbing) {

    

  }
  }



  public void setAllToColor(int r, int g, int b) {

    candle.setLEDs(r, g, b, 255, 8, 50);


  }
}
