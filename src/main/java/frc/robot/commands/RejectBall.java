// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.Constants.LoaderConstants;

public class RejectBall extends CommandBase {

  LoaderSubsystem loaderSubsystem;
  IntakeSubsystem intakeSubsystem;
  boolean isTeamRed;
  /** 
   * Checks if a ball is at the color sensor, what color it is, and what team the robot is on, then runs the loader to eject any enemy balls
   * 
   * @param isTeamRed False if the team is Blue, True if it is Red
  */
  public RejectBall(LoaderSubsystem loaderSubsystem, IntakeSubsystem intakeSubsystem, boolean isTeamRed) {
    this.loaderSubsystem = loaderSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.isTeamRed = isTeamRed;

    addRequirements(loaderSubsystem, intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if(intakeSubsystem.isBallLow() && LoaderConstants.checkColorRed(intakeSubsystem.getColorSensorRaw()) && isTeamRed) { 
      loaderSubsystem.runLoader(LoaderConstants.MAX_LOADER_SPEED);
    }

    if(intakeSubsystem.isBallLow() && LoaderConstants.checkColorBlue(intakeSubsystem.getColorSensorRaw()) && !isTeamRed) { 
      loaderSubsystem.runLoader(LoaderConstants.MAX_LOADER_SPEED);
    }
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
