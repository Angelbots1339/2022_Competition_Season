// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.candle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CandleSubsystem;

public class IdleAnimation extends CommandBase {

  private CandleSubsystem candleSubsystem;

  /** Creates a new idleAnimation. */
  public IdleAnimation(CandleSubsystem candleSubsystem) {

    this.candleSubsystem = candleSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

candleSubsystem.setToDefaultAnimation();

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
