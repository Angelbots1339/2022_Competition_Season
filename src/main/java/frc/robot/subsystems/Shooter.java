// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.ConstantBootstraps;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private WPI_TalonFX powerWheelRight = new WPI_TalonFX(RIGHT_POWER_WHEEL);
  private WPI_TalonFX powerWheelLeft = new WPI_TalonFX(LEFT_POWER_WHEEL);
  private WPI_TalonFX aimWheel = new WPI_TalonFX(AIM_WHEEL);

  private PIDController powerWheelPID = new PIDController(POWER_WHEEL_KP,POWER_WHEEL_KI, POWER_WHEEL_KD);

  private PIDController aimWheelPID = new PIDController(AIM_WHEEL_KP, AIM_WHEEL_KI, AIM_WHEEL_KD);
  private SimpleMotorFeedforward powerWheelFF = new SimpleMotorFeedforward(POWER_KS, POWER_KV, POWER_KA);

  private SimpleMotorFeedforward aimWheelFF = new SimpleMotorFeedforward(AIM_KS, AIM_KV, AIM_KA);
  private MotorControllerGroup powerWheelGroup = new MotorControllerGroup(powerWheelLeft, powerWheelRight);

  public Shooter() {
    powerWheelRight.configFactoryDefault();
    powerWheelLeft.configFactoryDefault();
    aimWheel.configFactoryDefault();

    powerWheelRight.setInverted(RIGHT_POWER_WHEEL_INVERTED);
    powerWheelLeft.setInverted(LEFT_POWER_WHEEL_INVERTED);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPowerWheelRPM(double speed) {
    double powerWheelRPM = (powerWheelRight.getSelectedSensorVelocity()/CLICKS_PER_ROT) * 600;
    powerWheelGroup.set(powerWheelPID.calculate(powerWheelRPM, speed));
  }

  public void setAimWheelRPM(double speed) {
    double aimWheelRPM = (aimWheel.getSelectedSensorVelocity()/CLICKS_PER_ROT) *600;
    aimWheel.set(aimWheelPID.calculate(aimWheelRPM, speed));
  }

}
