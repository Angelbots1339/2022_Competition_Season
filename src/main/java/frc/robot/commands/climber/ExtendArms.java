// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

import static frc.robot.Constants.ClimberConstants.*;

public class ExtendArms extends CommandBase {
  private PIDController leftExtend = new PIDController(EXTENDER_KP, 0, 0);
  private PIDController rightExtend = new PIDController(EXTENDER_KP, 0, 0);
  private ClimbingSubsystem climbingSubsystem;
  private double setpoint;

  /** Creates a new ExtendArms. */
  public ExtendArms(ClimbingSubsystem climbingSubsystem, boolean isRetract) {
    addRequirements(climbingSubsystem);
    setpoint = isRetract ? 0 : SLACK_LENGTH;

    this.climbingSubsystem = climbingSubsystem;
    leftExtend.setTolerance(EXTENDER_TOLERANCE);
    rightExtend.setTolerance(EXTENDER_TOLERANCE);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = (Math.abs(setpoint - climbingSubsystem.getLeftLength()) < EXTENDER_P_TOLERANCE)
        ? leftExtend.calculate(climbingSubsystem.getLeftLength(), setpoint)
        : EXTENDER_SPEED;
    double rightSpeed = (Math.abs(setpoint - climbingSubsystem.getRightLength()) < EXTENDER_P_TOLERANCE)
        ? rightExtend.calculate(climbingSubsystem.getRightLength(), setpoint)
        : EXTENDER_SPEED;
    climbingSubsystem.setExtensionSpeed(leftSpeed, rightSpeed); // TODO swictch pid to talonfx intrnal
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbingSubsystem.setExtensionSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rightExtend.atSetpoint() && leftExtend.atSetpoint();
  }
}
