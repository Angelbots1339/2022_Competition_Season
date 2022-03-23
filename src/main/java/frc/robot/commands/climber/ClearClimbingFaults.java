// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

public class ClearClimbingFaults extends CommandBase {
  /** Creates a new ClearClimbingFaults. */
  private ClimbingSubsystem climbingSubsystem;
  private boolean cleared = false;
  public ClearClimbingFaults(ClimbingSubsystem climbingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbingSubsystem);
    this.climbingSubsystem = climbingSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbingSubsystem.clearStickies();
    cleared = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cleared;
  }
}
