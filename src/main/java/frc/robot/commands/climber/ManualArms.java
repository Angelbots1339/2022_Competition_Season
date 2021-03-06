// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

public class ManualArms extends CommandBase {
  private final ClimbingSubsystem climbingSubsystem;
  private final DoubleSupplier extendVolts;
  private final DoubleSupplier rotateVolts;
  /** Creates a new RunArms. */
  public ManualArms(ClimbingSubsystem climbingSubsystem, DoubleSupplier extendVolts, DoubleSupplier rotateVolts) {
    addRequirements(climbingSubsystem);
    this.climbingSubsystem = climbingSubsystem;
    this.extendVolts = extendVolts;
    this.rotateVolts = rotateVolts;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbingSubsystem.setExtensionVolts(extendVolts.getAsDouble());
    climbingSubsystem.setRotationVolts(rotateVolts.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbingSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
