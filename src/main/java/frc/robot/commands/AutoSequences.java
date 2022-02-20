// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.NamedSequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public final class AutoSequences extends ArrayList<NamedSequentialCommandGroup> {
  public AutoSequences(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem, ShooterSubsystem shooterSubsystem) {
    super();
    // Example auto path
    // Turns on intake
    // Drives 1 meter
    // Turns off intake
    // Shoots low for 2 seconds
    this.add(
      "Example",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "1Meter"),
          new RunIntake(intakeSubsystem, loaderSubsystem)
        ),
        new ParallelDeadlineGroup(
          new WaitCommand(2),
          new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem, ShooterConstants.SHOOTER_PROFILE_LOW)
        )
      )
    );
  }

  private void add(String name, SequentialCommandGroup cmd) {
    this.add(new NamedSequentialCommandGroup(cmd, name));
  }
}
