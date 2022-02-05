// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

import static frc.robot.Constants.IntakeConstants.*;

/**
 * 
 * Runs intake, lower indexer, and if no ball is at the high color sensor it runs the loader
 */
public class RunIntake extends CommandBase {
  /** Creates a new RunIntake. */

  private final IntakeSubsystem intakeSubsystem;
  private final LoaderSubsystem loaderSubsystem;

  public RunIntake(IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, loaderSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.loaderSubsystem = loaderSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intakeSubsystem.runIntake(MAX_INTAKE_PERCENT);
    intakeSubsystem.runIndexerLow(MAX_INDEXER_PERCENT);

    if(!intakeSubsystem.isBallLow()) {

      loaderSubsystem.runLoader(MAX_INDEXER_PERCENT);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.Disable();
    loaderSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
