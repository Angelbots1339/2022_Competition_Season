// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

import static frc.robot.Constants.ClimberConstants.*;

public class ExtendArms extends CommandBase {
  private ClimbingSubsystem climbingSubsystem;
  private boolean isRetract;

  private double direction;
  /** Creates a new ExtendArms. */
  public ExtendArms(ClimbingSubsystem climbingSubsystem, boolean isRetract) {
    addRequirements(climbingSubsystem);
    this.climbingSubsystem = climbingSubsystem;
    this.isRetract = isRetract;
    direction = isRetract? 1 : -1;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    if (inBounds(climbingSubsystem.getLeftLength())) {
      climbingSubsystem.setExtensionSpeedLeft(EXTENDER_PERCENT_MAX * direction);
    }

    if (inBounds(climbingSubsystem.getRightLength())) {
      climbingSubsystem.setExtensionSpeedRight(EXTENDER_PERCENT_MAX * direction);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if(!inBounds(climbingSubsystem.getLeftLength())){
      climbingSubsystem.setExtensionSpeedLeft(0);
    }

    if (!inBounds(climbingSubsystem.getRightLength())) {
      climbingSubsystem.setExtensionSpeedRight(0);
    } 

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbingSubsystem.setExtensionSpeed(0, 0);
  }

  public boolean inBounds(double length) {
    if(isRetract){
      return length < 0.8;
    }
    else{
      return length > 0;
    }
    // return true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !inBounds(climbingSubsystem.getLeftLength()) && !inBounds(climbingSubsystem.getRightLength());
  }
}
