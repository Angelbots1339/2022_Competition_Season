// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.networktables.NetworkTableInstance;


import frc.robot.RobotContainer;
import frc.robot.Constants.CandleConstants;

/** Add your docs here. */
public class Candle {
    private static Candle CANDLE = null;

    private int climbIndx = 0;

    private int climbStages = 14;

    private CANdle candle = new CANdle(CandleConstants.CANDLE_ID);


    public enum LEDState {
        Fire,
        Idle,
        Intake,
        Climbing,
        ManualClimbing,
        ReverseIntake,
        Reject,
        Disabled,
        PreMatch,
        TestMode,
        Off
    }

    private LEDState currentState = LEDState.Off;

    private DoubleSupplier leftTrigger;
    private DoubleSupplier rightTrigger;
    private DoubleSupplier rightYAxsis;

    private DoubleSupplier LSpeed;
    private DoubleSupplier RSpeed;

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

    public void incrementClimb() {
        ++climbIndx;
    }

    public void changeLedState(LEDState state) {

        for (int i = 0; i < 10; ++i) {
            candle.clearAnimation(i);
        }
        candle.setLEDs(0, 0, 0, 0, 8, 128);
        switch (state) {
            case Fire:
                candle.animate(new FireAnimation(1, 0.7, 32, 0.8, 0.4, false, 8), 1);
                candle.animate(new FireAnimation(1, 0.7, 32, 0.8, 0.4, true, 120 + 8 - 32), 2);
                candle.animate(new FireAnimation(1, 0.7, 13, 0.8, 0.6, false, 68), 3);
                candle.animate(new FireAnimation(1, 0.7, 14, 0.8, 0.3, true, 68 - 16), 4);
                currentState = LEDState.Fire;
                // candle.animate(new TwinkleAnimation(255, 18, 213, 0, .1, 120 + 8 - 58,
                // TwinklePercent.Percent30, 58), 3);
                break;
            case Idle:

                currentState = LEDState.Idle;

                break;
            case Intake:
                candle.animate(new ColorFlowAnimation(128, 20, 60, 0, 0.9, 60, Direction.Forward, 8), 1);
                candle.animate(new ColorFlowAnimation(128, 20, 60, 0, 0.9, 60, Direction.Backward, 68), 2);

                currentState = LEDState.Intake;

                break;
            case Climbing:
                // candle.animate(new LarsonAnimation(255, 140, 0, 0, 0.7, 120, BounceMode.Back, 3, 8), 1);
                currentState = LEDState.Climbing;

                break;
            case ReverseIntake:

                candle.animate(new ColorFlowAnimation(255, 20, 0, 0, 0.9, 60, Direction.Backward, 8), 1);
                candle.animate(new ColorFlowAnimation(255, 20, 0, 0, 0.9, 60, Direction.Forward, 68), 2);

                currentState = LEDState.ReverseIntake;
                break;
            case Disabled:

                candle.animate(new TwinkleAnimation(255, 255, 255, 0, 0.4, 120, TwinklePercent.Percent30, 8), 1);

                currentState = LEDState.Disabled;
                break;
            case Reject:

                candle.animate(new ColorFlowAnimation(255, 20, 0, 0, 1, 60, Direction.Backward, 8), 1);
                candle.animate(new ColorFlowAnimation(255, 20, 0, 0, 1, 60, Direction.Forward, 68), 2);

                currentState = LEDState.Reject;
                break;
            case PreMatch:

                // candle.animate(new LarsonAnimation(255, 0, 255, 0, 0.01, 120,
                // BounceMode.Back, 7, 8), 1);
                // candle.animate(new ColorFlowAnimation(255, 0, 255, 0, 0.01, 120,
                // Direction.Forward, 8), 1);
                // candle.animate(new SingleFadeAnimation(255, 0, 0, 0, 0.1, 120, 8), 1);

                currentState = LEDState.PreMatch;
                break;
            case TestMode:
                candle.animate(new SingleFadeAnimation(200, 70, 0, 0, 0.1, 120, 8), 1);

                currentState = LEDState.TestMode;
                break;
            case Off:

                currentState = LEDState.Off;
                break;
            default:
                break;
        }
    }

    public void periodic() {
        if (currentState == LEDState.Climbing) {
            candle.setLEDs(0, 255, 0, 255, 8, (int)(60/climbStages)*(climbIndx));
            candle.setLEDs(0, 255, 0, 255, 128 - (int)(60/climbStages)*(climbIndx), (int)(60/climbStages)*(climbIndx));
        }
        if (currentState == LEDState.ManualClimbing) {
            double rotation = rightYAxsis.getAsDouble();
            double height = leftTrigger.getAsDouble() - rightTrigger.getAsDouble();
            System.out.println(rotation);

            if (rotation > 0.01) {
                candle.setLEDs(0, 255, 0, 255, 8, (int) (60 * rotation));
                candle.setLEDs(0, 0, 0, 0, 8 + (int) (60 * rotation), 60 - (int) (60 * rotation));
            }
            if (height > 0.01) {
                candle.setLEDs(0, 255, 0, 255, 128 - (int) (60 * height), (int) (60 * height));
                candle.setLEDs(0, 0, 0, 0, 68, 60 - (int) (60 * height));
            }

            if (rotation < -0.01) {
                candle.setLEDs(255, 0, 0, 255, 8, (int) (60 * -rotation));
                candle.setLEDs(0, 0, 0, 0, 8 + (int) (60 * -rotation), 60 - (int) (60 * -rotation));
            }
            if (height < -0.01) {
                candle.setLEDs(255, 0, 0, 255, 128 - (int) (60 * -height), (int) (60 * -height));
                candle.setLEDs(0, 0, 0, 0, 68, 60 - (int) (60 * -height));
            }

            if (0.01 >= rotation && -0.1 <= rotation) {
                candle.setLEDs(0, 0, 0, 0, 8, 60);
            }

            if (0.01 >= height && -0.1 <= height) {
                candle.setLEDs(0, 0, 0, 0, 68, 60);
            }
        }

        if (currentState == LEDState.Idle) {

            // double leftSpeed = MathUtil.clamp(fwd.getAsDouble() - rot.getAsDouble(), -1,
            // 1);
            // double rightSpeed = MathUtil.clamp(fwd.getAsDouble() + rot.getAsDouble(), -1,
            // 1);

            double leftSpeed = LSpeed.getAsDouble() / 3.5;
            double rightSpeed = RSpeed.getAsDouble() / 3.5;

            if (rightSpeed > 0.01) {
                candle.setLEDs(0, 255, 0, 255, 8, (int) (60 * rightSpeed));
                candle.setLEDs(0, 0, 0, 0, 8 + (int) (60 * rightSpeed), 60 - (int) (60 * rightSpeed));
            }
            if (leftSpeed > 0.01) {
                candle.setLEDs(0, 255, 0, 255, 128 - (int) (60 * leftSpeed), (int) (60 * leftSpeed));
                candle.setLEDs(0, 0, 0, 0, 68, 60 - (int) (60 * leftSpeed));
            }

            if (rightSpeed < -0.01) {
                candle.setLEDs(255, 0, 0, 255, 8, (int) (60 * -rightSpeed));
                candle.setLEDs(0, 0, 0, 0, 8 + (int) (60 * -rightSpeed), 60 - (int) (60 * -rightSpeed));
            }
            if (leftSpeed < -0.01) {
                candle.setLEDs(255, 0, 0, 255, 128 - (int) (60 * -leftSpeed), (int) (60 * -leftSpeed));
                candle.setLEDs(0, 0, 0, 0, 68, 60 - (int) (60 * -leftSpeed));
            }

            if (0.01 >= rightSpeed && -0.1 <= rightSpeed) {
                candle.setLEDs(0, 0, 0, 0, 8, 60);
            }

            if (0.01 >= leftSpeed && -0.1 <= leftSpeed) {
                candle.setLEDs(0, 0, 0, 0, 68, 60);
            }
        }





        if (currentState == LEDState.PreMatch) {

            if (NetworkTableInstance.getDefault().isConnected()) {
                RobotContainer.setTeamColor();

                if (RobotContainer.getTeamColor()) {
                    // Red team

                    candle.animate(new SingleFadeAnimation(255, 0, 0, 0, 0.1, 120, 8), 1);

                } else {
                    // Blue Team
                    candle.animate(new SingleFadeAnimation(0, 0, 255, 0, 0.1, 120, 8), 1);

                }
            } else {

                candle.animate(new SingleFadeAnimation(255, 0, 255, 0, 0.1, 120, 8), 1);
            }
        }

    }

    public void setRobotSpeed(DoubleSupplier LSpeed, DoubleSupplier RSpeed) {
        this.RSpeed = RSpeed;
        this.LSpeed = LSpeed;
    }
    public void setClimbSuppliers(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger, DoubleSupplier rightYAxsis) {
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.rightYAxsis = rightYAxsis;
    }

    public double getVbat() {
        return candle.getBusVoltage();
    }

    public double get5V() {
        return candle.get5VRailVoltage();
    }

    public double getCurrent() {
        return candle.getCurrent();
    }

    public double getTemperature() {
        return candle.getTemperature();
    }

    public void configBrightness(double percent) {
        candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        candle.configStatusLedState(offWhenActive, 0);
    }

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
