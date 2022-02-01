// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

public class RotateToAngles extends CommandBase {
  private ClimbingSubsystem climbingSubsystem;
  private double target;
  /** Creates a new RotateToAngles. */
  public RotateToAngles(ClimbingSubsystem climbingSubsystem, double target) {
    addRequirements(climbingSubsystem);
    this.climbingSubsystem = climbingSubsystem;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO ask nick wether we need to go to a specific angle or just to the limit swich
    //climbingSubsystem.setRotationSpeed(target > climbingSubsystem.getRightAngle(), 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
