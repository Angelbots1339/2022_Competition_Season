// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.TurnConstants;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */

  DriveSubsystem driveSubsystem;
  double angle;

  public TurnToAngle(DriveSubsystem driveSubsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveSubsystem = driveSubsystem;
    this.angle = angle;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(driveSubsystem.getHeading().getDegrees() - angle) > 180) {
      driveSubsystem.tankDriveVolts(-3, 3);

    } else {
      driveSubsystem.tankDriveVolts(3, -3);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getHeading().getDegrees() - angle) > TurnConstants.TURN_THRESHOLD;
  }
}
