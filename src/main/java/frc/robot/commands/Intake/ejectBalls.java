// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.LoaderConstants.*;

import frc.robot.Constants.LoaderConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

public class ejectBalls extends CommandBase {
  /** Creates a new ejectBalls. */
  private IntakeSubsystem intakeSubsystem;
  private LoaderSubsystem loaderSubsystem;
  public ejectBalls(IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.loaderSubsystem = loaderSubsystem;
    addRequirements(loaderSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.runIndexerLow(-MAX_INDEXER_PERCENT);
    intakeSubsystem.runIntake(-MAX_INTAKE_PERCENT);
    loaderSubsystem.runLoader(-MAX_LOADER_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.disable();
    loaderSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
