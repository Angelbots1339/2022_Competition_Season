// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.IntakeConstants.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

public class RetractIntake extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  /** Creates a new DeployIntake. */
  public RetractIntake(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.runRightDeployMotorsVolts(-INTAKE_RETRACT_MAX_VOLTS);
    intakeSubsystem.runLeftDeployMotorsVolts(-INTAKE_RETRACT_MAX_VOLTS);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Feed Forward to get both motors to their setpoints
    if (Math.abs(intakeSubsystem.getRightDeployMotorPosition() - RETRACTION_SETPOINT) > RETRACTION_THRESHOLD) {
      intakeSubsystem.runRightDeployMotorsVolts(-INTAKE_RETRACT_MAX_VOLTS);
    } else {
      intakeSubsystem.runRightDeployMotorsVolts(0);
    }
    if (Math.abs(intakeSubsystem.getLeftDeployMotorPosition() - RETRACTION_SETPOINT) > RETRACTION_THRESHOLD) {
      intakeSubsystem.runLeftDeployMotorsVolts(-INTAKE_RETRACT_MAX_VOLTS);
    } else {
      intakeSubsystem.runLeftDeployMotorsVolts(0);
    }
    }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(intakeSubsystem.getRightDeployMotorPosition() - RETRACTION_SETPOINT) > RETRACTION_THRESHOLD;
  }
}
