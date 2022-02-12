// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

import static frc.robot.Constants.IntakeConstants.*;

public class LoadShooter extends CommandBase {

  LoaderSubsystem loaderSubsystem;
  IntakeSubsystem intakeSubsystem;
  public LoadShooter(LoaderSubsystem loaderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.loaderSubsystem = loaderSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(loaderSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loaderSubsystem.runLoader(MAX_INDEXER_PERCENT);
    intakeSubsystem.runIndexerLow(MAX_INDEXER_PERCENT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    loaderSubsystem.disable();
    intakeSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}