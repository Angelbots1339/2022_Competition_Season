// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private WPI_TalonFX powerWheelRight = new WPI_TalonFX(RIGHT_POWER_WHEEL, Constants.CANIVORE_NAME);
  private WPI_TalonFX powerWheelLeft = new WPI_TalonFX(LEFT_POWER_WHEEL, Constants.CANIVORE_NAME);
  private WPI_TalonFX aimWheel = new WPI_TalonFX(AIM_WHEEL, Constants.CANIVORE_NAME);

  private PIDController powerWheelPID = new PIDController(POWER_WHEEL_KP,POWER_WHEEL_KI, POWER_WHEEL_KD);

  private PIDController aimWheelPID = new PIDController(AIM_WHEEL_KP, AIM_WHEEL_KI, AIM_WHEEL_KD);

  private SimpleMotorFeedforward powerWheelFF = new SimpleMotorFeedforward(POWER_KS, POWER_KV, POWER_KA);
  private SimpleMotorFeedforward aimWheelFF = new SimpleMotorFeedforward(AIM_KS, AIM_KV, AIM_KA);
  
  //private MotorControllerGroup powerWheelGroup = new MotorControllerGroup(powerWheelLeft, powerWheelRight);

  private ShuffleboardTab tab = Shuffleboard.getTab("ShooterSystem");

  //TODO delete 
  private double aimPID = 0;
  private double powerPID = 0;

  public ShooterSubsystem() {
    //powerWheelRight.configFactoryDefault();
    //powerWheelLeft.configFactoryDefault();
    //aimWheel.configFactoryDefault();

    powerWheelRight.setInverted(RIGHT_POWER_WHEEL_INVERTED);
    powerWheelLeft.setInverted(LEFT_POWER_WHEEL_INVERTED);

    // powerWheelLeft.clearStickyFaults();
    // powerWheelRight.clearStickyFaults();
    // aimWheel.clearStickyFaults();

    powerWheelPID.setTolerance(POWER_WHEEL_TOLERANCE);
    aimWheelPID.setTolerance(AIM_WHEEL_TOLERANCE);

    tab.add(aimWheelPID);
    tab.add(powerWheelPID);
    tab.addNumber("Aim Wheel Speed", () -> getAimRPM());
    tab.addNumber("Power Wheel Speed", () -> getPowerRPM());
    tab.addNumber("Aim PID Out", () -> aimPID);
    tab.addNumber("Power PID Out", () -> powerPID);
    tab.addNumber("Power FeedForward Out", () -> powerPID);
    tab.addNumber("Power FeedForward Out", () -> powerPID);

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * 
   * @param speed RPM (Rotations per Minute)
   */
  public void setPowerWheelRPM(double speed) {
    powerPID = powerWheelPID.calculate(getPowerRPM(), speed);
    double powerFeedForward = powerWheelFF.calculate(speed / 60);
    
    setPowerWheelPercentage(powerPID + powerFeedForward);
    powerWheelLeft.setVoltage(powerFeedForward + powerPID);
    powerWheelRight.setVoltage(powerFeedForward + powerPID);
  }
  public void setPowerWheelPercentage(double speed){

      powerWheelLeft.set(speed);
      powerWheelRight.set(speed);
  }

  public void setAimWheelRPM(double speed) {
    aimPID = aimWheelPID.calculate(getAimRPM(), speed);
    double aimWheelFeedForward = aimWheelFF.calculate(speed / 60);

    aimWheel.set(aimPID + aimWheelFeedForward);
  }
  public void setAimWheelPercentage(double speed) {
    aimWheel.set(speed);
  }
  /**
   * Disable all motors
   */
  public void disable() {
    powerWheelLeft.set(0);
    powerWheelRight.set(0);
    aimWheel.set(0);
  }

  //GET

  public boolean isAtSetpoint() {
    
    SmartDashboard.putBoolean("Power at Setpoing", powerWheelPID.atSetpoint());
    SmartDashboard.putBoolean("Aim at Setpoint", aimWheelPID.atSetpoint());
    return powerWheelPID.atSetpoint() && aimWheelPID.atSetpoint();

  }

  public double getPowerRPM(){
    return (powerWheelRight.getSelectedSensorVelocity() / CLICKS_PER_ROT) * 600;
  }
  public double getAimRPM(){
    return (aimWheel.getSelectedSensorVelocity()/CLICKS_PER_ROT) * 600;
  }


}
