// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowComandGenerationBellCurve extends SequentialCommandGroup {
  /** Creates a new FollowComandGenerationBellCurve. */
  public FollowComandGenerationBellCurve(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(driveSubsystem);

    addCommands(
        new FollowTrajectory(driveSubsystem, FollowTrajectory.getAutoTrajectory1()),
        new ParallelRaceGroup(
            new WaitCommand(4),
            new RunCommand(() -> driveSubsystem.tankDriveVolts(0.0, 0.0), driveSubsystem)),
        new FollowTrajectory(driveSubsystem, FollowTrajectory.getAutoTrajectory2()));
  }
  
}
