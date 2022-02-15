// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.BooleanSupplier;

public class AutoClimbMoveArms extends CommandBase {
  private ClimbingSubsystem climbingSubsystem;
  private double extenderVelocity;
  private double rotatorVelocity;

  private double angleSetpoint;
  private double lengthSetpoint;

  /** Creates a new ExtendArms. */
  /**
   * 
   * @param climbingSubsystem
   * @param lengthSetpoint    // m
   * @param angleSetpoint     // degrees
   */
  public AutoClimbMoveArms(ClimbingSubsystem climbingSubsystem, double lengthSetpoint, double angleSetpoint) {
    addRequirements(climbingSubsystem);

    // TODO check if negative / positive is flipped
    // Negative is out, positive is in

    extenderVelocity = MAX_EXTENDER_SPEED * Math.signum(lengthSetpoint - climbingSubsystem.getRightLength());
    rotatorVelocity = MAX_ROTATOR_SPEED * Math.signum(angleSetpoint - climbingSubsystem.getRightAngle());

    this.climbingSubsystem = climbingSubsystem;
    this.angleSetpoint = angleSetpoint;
    this.lengthSetpoint = lengthSetpoint;
  }

  public AutoClimbMoveArms(ClimbingSubsystem climbingSubsystem, double lengthSetpoint) {
    this(climbingSubsystem, lengthSetpoint, climbingSubsystem.getRightAngle());
  }

  public AutoClimbMoveArms(double angleSetpoint, ClimbingSubsystem climbingSubsystem) {
    this(climbingSubsystem, climbingSubsystem.getRightLength(), angleSetpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //climbingSubsystem.setExtensionSpeed(extenderVelocity);
    //climbingSubsystem.setRotationSpeed(rotatorVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isWithinThreshold(climbingSubsystem.getRightLength(), lengthSetpoint)) {
      climbingSubsystem.setRightExtensionVolts(0);
    }
    if (isWithinThreshold(climbingSubsystem.getRightAngle(), angleSetpoint)) {
      climbingSubsystem.setRightRotationVolts(0);
    }

    if (isWithinThreshold(climbingSubsystem.getLeftAngle(), angleSetpoint)) {
      climbingSubsystem.setLeftRotationVolts(0);
    }
    if (isWithinThreshold(climbingSubsystem.getLeftLength(), lengthSetpoint)) {
      climbingSubsystem.setLeftExtensionVolts(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbingSubsystem.setRotationVolts(0);
    climbingSubsystem.setExtensionSpeedSimpleVolts(() -> 0);
  }

  private boolean isWithinThreshold(double length, double setpoint) {
    return Math.abs(length - setpoint) < EXTENDER_SETPOINT_THRESHOLD;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isWithinThreshold(climbingSubsystem.getRightLength(), lengthSetpoint)
        && isWithinThreshold(climbingSubsystem.getRightAngle(), angleSetpoint)
        && isWithinThreshold(climbingSubsystem.getLeftAngle(), angleSetpoint)
        && isWithinThreshold(climbingSubsystem.getLeftLength(), lengthSetpoint);

  }
}