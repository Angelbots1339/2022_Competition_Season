// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ClimberConstants.*;
import frc.robot.subsystems.ClimbingSubsystem;

public class RotateClimberBack extends CommandBase {
  /** Creates a new RotateClimberBack. */
  private ClimbingSubsystem climbingSubsystem;
  private PIDController leftController = new PIDController(ROTATOR_KP, 0, 0);
  private PIDController rightController = new PIDController(ROTATOR_KP, 0, 0);

  public RotateClimberBack(ClimbingSubsystem climbingSubsystem) {
    this.climbingSubsystem = climbingSubsystem;
    addRequirements(climbingSubsystem);
    leftController.setTolerance(ROTATOR_ANGLE_TOLERANCE);
    rightController.setTolerance(ROTATOR_ANGLE_TOLERANCE);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = climbingSubsystem.getLeftAngle() < ROTATOR_ANGLE_P_TOLERANCE 
        ? leftController.calculate(climbingSubsystem.getLeftAngle(), 0)
        : -ROTATOR_PERCENT_MAX;
    double rightSpeed = climbingSubsystem.getRightAngle() < ROTATOR_ANGLE_P_TOLERANCE
        ? rightController.calculate(climbingSubsystem.getRightAngle(), 0)
        : -ROTATOR_PERCENT_MAX;

    climbingSubsystem.setExtensionSpeed(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftController.atSetpoint() && rightController.atSetpoint();
  }
}
