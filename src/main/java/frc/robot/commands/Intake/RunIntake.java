// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.LoaderConstants.*;

import java.util.function.BooleanSupplier;

/**
 * Runs balls up the intake to the color sensor
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
  public void initialize() {

    // Start motors at speed safe for deploy
    loaderSubsystem.runLoader(MAX_LOADER_INTAKE_SPEED);
    intakeSubsystem.runIntake(INTAKE_DEPLOY_SPEED);
    intakeSubsystem.runIndexerLow(INTAKE_DEPLOY_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Feed Forward to get both motors to their setpoints
    if (Math.abs(intakeSubsystem.getRightDeployMotorPosition() - DEPLOY_SETPOINT) > RETRACTION_THRESHOLD) {
      intakeSubsystem.runRightDeployMotorsVolts(INTAKE_RETRACT_MAX_VOLTS);
    } else {
      intakeSubsystem.runRightDeployMotorsVolts(0);
    }
    if (Math.abs(intakeSubsystem.getLeftDeployMotorPosition() - DEPLOY_SETPOINT) > RETRACTION_THRESHOLD) {
      intakeSubsystem.runLeftDeployMotorsVolts(INTAKE_RETRACT_MAX_VOLTS);
    } else {
      intakeSubsystem.runLeftDeployMotorsVolts(0);
    }

    // Runs intake and indexer if both motors have reached their setpoints
    if (Math.abs(intakeSubsystem.getRightDeployMotorPosition() - DEPLOY_SETPOINT) < RETRACTION_THRESHOLD
        && Math.abs(intakeSubsystem.getLeftDeployMotorPosition() - DEPLOY_SETPOINT) < RETRACTION_THRESHOLD) {

      intakeSubsystem.runDeployMotorsVolts(0);

      loaderSubsystem.runLoader(MAX_LOADER_INTAKE_SPEED);

      intakeSubsystem.runIntake(MAX_INTAKE_PERCENT);
      intakeSubsystem.runIndexerLow(MAX_INDEXER_PERCENT);

      if (intakeSubsystem.isBallLow()) {
        loaderSubsystem.runLoader(0);
      }

    }
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
