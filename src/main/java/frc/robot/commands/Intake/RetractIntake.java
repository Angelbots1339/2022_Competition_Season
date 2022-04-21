// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.IntakeConstants.*;
import frc.robot.subsystems.IntakeSubsystem;

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
    intakeSubsystem.runIntake(INTAKE_DEPLOY_SPEED);
    intakeSubsystem.runIndexerLow(INTAKE_DEPLOY_SPEED);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setDeployMotorsVolts(
      isLeftRetracted() ? 0 : -INTAKE_RETRACT_VOLTS,
      isRightRetracted() ? 0 : -INTAKE_RETRACT_VOLTS);

      if(isLeftRetracted() && isRightRetracted()){
        intakeSubsystem.runIntake(0);
        intakeSubsystem.runIndexerLow(0);

      } else {
        intakeSubsystem.runIntake(INTAKE_DEPLOY_SPEED);
        intakeSubsystem.runIndexerLow(INTAKE_DEPLOY_SPEED);

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
    // Don't finish, default command
    return isLeftRetracted() && isRightRetracted()
    ;
  }

  private boolean isLeftRetracted() {
    return intakeSubsystem.getLeftPosition() <= RETRACTION_SETPOINT;
  }
  private boolean isRightRetracted() {
    return intakeSubsystem.getRightPosition() <= RETRACTION_SETPOINT;
  }
}
