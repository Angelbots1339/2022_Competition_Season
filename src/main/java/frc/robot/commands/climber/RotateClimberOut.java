// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ClimberConstants.*;

import frc.robot.subsystems.ClimbingSubsystem;

public class RotateClimberOut extends CommandBase {
  private ClimbingSubsystem climbingSubsystem;
  /** Creates a new RotateToAngles. */
  public RotateClimberOut(ClimbingSubsystem climbingSubsystem) {
    addRequirements(climbingSubsystem);
    this.climbingSubsystem = climbingSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbingSubsystem.setRotationSpeed(climbingSubsystem.getLeftRotatorLimit()? 0 : ROTATOR_SPEED , climbingSubsystem.getRightRotatorLimit()? 0 : ROTATOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbingSubsystem.setRotationSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbingSubsystem.getLeftRotatorLimit() && climbingSubsystem.getRightRotatorLimit();
  }
}
