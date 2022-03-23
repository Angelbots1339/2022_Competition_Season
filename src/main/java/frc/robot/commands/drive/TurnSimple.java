// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Turns robot for a specified time
 */
public class TurnSimple extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Timer timer;
  private final double time;
  private final double rightVolts;

  /**
   * @param driveSubsystem
   * @param time Time to turn (s)
   * @param rightVolts Volts on the right side of the drive train (left is inverted)
   */
  public TurnSimple(DriveSubsystem driveSubsystem, double time, double rightVolts) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.timer = new Timer();
    this.time = time;
    this.rightVolts = rightVolts;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.tankDriveVolts(-rightVolts, rightVolts);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
