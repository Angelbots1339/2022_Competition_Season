// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
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
    intakeSubsystem.runDeployMotorsVolts(-IntakeConstants.INTAKE_RETRACT_MAX_VOLTS);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(intakeSubsystem.getRightDeployMotorPosition() - IntakeConstants.RETRACTION_SETPOINT) > IntakeConstants.RETRACTION_THRESHOLD){

      intakeSubsystem.runDeployMotorsVolts(IntakeConstants.INTAKE_RETRACT_MAX_VOLTS);
  
      } else{

        intakeSubsystem.runDeployMotorsVolts(0);

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
    return Math.abs(intakeSubsystem.getRightDeployMotorPosition() - IntakeConstants.RETRACTION_SETPOINT) > IntakeConstants.RETRACTION_THRESHOLD;
  }
}
