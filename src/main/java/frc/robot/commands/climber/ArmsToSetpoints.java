// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

import static frc.robot.Constants.ClimberConstants.*;

public class ArmsToSetpoints extends CommandBase {
  private ClimbingSubsystem climbingSubsystem;
  private double currentRightExtender = 0;
  private double currentRightRotator = 0;
  private double currentLeftExtender = 0;
  private double currentLeftRotator = 0;

  private final double angleSetpoint;
  private final double lengthSetpoint;

  /** Creates a new ExtendArms. */
  /**
   * 
   * @param climbingSubsystem
   * @param lengthSetpoint    // m
   * @param angleSetpoint     // degrees
   */
  public ArmsToSetpoints(ClimbingSubsystem climbingSubsystem, double lengthSetpoint, double angleSetpoint) {
    addRequirements(climbingSubsystem);

    this.climbingSubsystem = climbingSubsystem;
    this.angleSetpoint = angleSetpoint;
    this.lengthSetpoint = lengthSetpoint;
  }

  public ArmsToSetpoints(ClimbingSubsystem climbingSubsystem, double lengthSetpoint) {
    this(climbingSubsystem, lengthSetpoint, climbingSubsystem.getRightAngle());
  }

  public ArmsToSetpoints(double angleSetpoint, ClimbingSubsystem climbingSubsystem) {
    this(climbingSubsystem, climbingSubsystem.getRightLength(), angleSetpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stop individual movement once arms reach setpoints.
    // Restart movement if arms have moved out of setpoints.
    // Only call when changed to avoid high traffic.
    double leftExtendDesired = desiredOutput(lengthSetpoint, climbingSubsystem.getLeftLength(), MAX_EXTENDER_VOLTS, EXTENDER_SETPOINT_THRESHOLD);
    double rightExtendDesired = desiredOutput(lengthSetpoint, climbingSubsystem.getRightLength(), MAX_EXTENDER_VOLTS, EXTENDER_SETPOINT_THRESHOLD);
    double leftRotateDesired = desiredOutput(angleSetpoint, climbingSubsystem.getLeftAngle(), MAX_ROTATOR_VOLTS, ROTATION_SETPOINT_THRESHOLD);
    double rightRotateDesired = desiredOutput(angleSetpoint, climbingSubsystem.getRightAngle(), MAX_ROTATOR_VOLTS, ROTATION_SETPOINT_THRESHOLD);
    if(leftExtendDesired != currentLeftExtender) {
      currentLeftExtender = leftExtendDesired;
      climbingSubsystem.setLeftExtensionVolts(currentLeftExtender);
    }
    if(rightExtendDesired != currentRightExtender) {
      currentRightExtender = rightExtendDesired;
      climbingSubsystem.setLeftExtensionVolts(currentRightExtender);
    }
    if(leftRotateDesired != currentLeftRotator) {
      currentLeftRotator = leftRotateDesired;
      climbingSubsystem.setLeftRotationVolts(currentLeftRotator);
    }
    if(rightRotateDesired != currentRightRotator) {
      currentRightRotator = rightRotateDesired;
      climbingSubsystem.setLeftExtensionVolts(currentRightRotator);
    }
  }

  private double desiredOutput(double setpoint, double position, double maxOutput, double threshold) {
    if(isWithinThreshold(setpoint, position, threshold)) return 0;
    return maxOutput * Math.signum(setpoint - position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbingSubsystem.setRotationVolts(0);
    climbingSubsystem.setExtensionVolts(0);
  }

  private boolean isWithinThreshold(double setpoint, double position, double threshold) {
    return Math.abs(position - position) < threshold;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isWithinThreshold(climbingSubsystem.getRightLength(), lengthSetpoint, EXTENDER_SETPOINT_THRESHOLD)
        && isWithinThreshold(climbingSubsystem.getRightAngle(), angleSetpoint, ROTATION_SETPOINT_THRESHOLD)
        && isWithinThreshold(climbingSubsystem.getLeftAngle(), angleSetpoint, ROTATION_SETPOINT_THRESHOLD)
        && isWithinThreshold(climbingSubsystem.getLeftLength(), lengthSetpoint, EXTENDER_SETPOINT_THRESHOLD);
  }
}