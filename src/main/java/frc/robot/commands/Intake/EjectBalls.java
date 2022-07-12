// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.LoaderConstants.*;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.utils.Candle;
import frc.robot.utils.Candle.LEDState;

/**
 * Run the intake backwards
 */
public class EjectBalls extends CommandBase {
  /** Creates a new ejectBalls. */
  private IntakeSubsystem intakeSubsystem;
  private LoaderSubsystem loaderSubsystem;
  public EjectBalls(IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.loaderSubsystem = loaderSubsystem;
    addRequirements(loaderSubsystem, intakeSubsystem);
  }

  @Override
  public void initialize() {

    Candle.getInstance().changeLedState(LEDState.ReverseIntake);

  }

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
    Candle.getInstance().changeLedState(LEDState.Idle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
