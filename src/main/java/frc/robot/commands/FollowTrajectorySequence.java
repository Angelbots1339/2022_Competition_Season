// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowTrajectorySequence extends SequentialCommandGroup {
  public FollowTrajectorySequence(DriveSubsystem m_driveSubsystem) {
    addRequirements(m_driveSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        FollowTrajectory.followTrajectoryFromJSON(m_driveSubsystem, "Unnamed_0"),
        new ParallelRaceGroup(
            new WaitCommand(10),
            new RunCommand(() -> m_driveSubsystem.tankDriveVolts(0.0, 0.0), m_driveSubsystem)),
        FollowTrajectory.followTrajectoryFromJSON(m_driveSubsystem, "Unnamed"));
  }

}
