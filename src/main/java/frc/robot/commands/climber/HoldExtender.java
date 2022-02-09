// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;
import static frc.robot.Constants.ClimberConstants.*;

public class HoldExtender extends CommandBase {

  

  private PIDController leftController = new PIDController(LEFT_EXTENDER_HOLD_KP, 0, 0);
  private PIDController rightController = new PIDController(RIGHT_EXTENDER_HOLD_KP, 0, 0);

  private ClimbingSubsystem climbingSubsystem;

  /** Creates a new holdExtender. */
  public HoldExtender(ClimbingSubsystem climbingSubsystem) {

    addRequirements(climbingSubsystem);

    this.climbingSubsystem = climbingSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

 

    rightController.setSetpoint(climbingSubsystem.getRightLength());
    leftController.setSetpoint(climbingSubsystem.getLeftLength());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    climbingSubsystem.setLeftExtensionVolts(leftController.calculate(climbingSubsystem.getLeftLength()));
    climbingSubsystem.setRightExtensionVolts(rightController.calculate(climbingSubsystem.getRightLength()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
