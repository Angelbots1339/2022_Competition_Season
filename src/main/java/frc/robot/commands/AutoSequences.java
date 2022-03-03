// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static frc.robot.Constants.ShooterConstants;
import static frc.robot.Constants.AutoConstants.*;
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

    private final DriveSubsystem driveSubsystem;

    public AutoSequences(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem,
            LoaderSubsystem loaderSubsystem,
            ShooterSubsystem shooterSubsystem) {
        super();
        this.driveSubsystem = driveSubsystem;
        // Example auto path
        // Turns on intake
        // Drives 1 meter
        // Turns off intake
        // Shoots low for 2 seconds
        this.add(
                "Example",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "2Meter"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),
                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW))

                ),
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "2Meter").getStartPose2d());

        // Shoots ball and drives past line
        this.add(
                "1 Ball",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW)),
                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "1BForward")),
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "1BForward").getStartPose2d());

        // Grabs second ball and shoots
        this.add(
                "2 Ball",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "2BGrab"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "2BShoot"),

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW))),
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "2BGrab").getStartPose2d());

        // Grabs a different second ball and shoots
        this.add(
                "2 Ball Alt 1",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BGrab1stSet"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BShoot1stSet"),

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW))

                ),
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BGrab1stSet").getStartPose2d());
        // Grabs another different second ball and shoots
        this.add(
                "2 Ball Alt 2",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "2BAlt2Grab"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "2BAlt2Shoot"),

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW))),
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "2BAlt2Shoot").getStartPose2d());

        // Shoots first ball, grabs second and third, then shoots
        this.add(
                "3 Ball",
                new SequentialCommandGroup(

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW)),

                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "3BGrab"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "3BShoot"),

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW))),

                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "3BGrab").getStartPose2d());

        // Grabs first ball, shoots, grabs 2nd ball, shoots
        this.add(
                "3 Ball Alt1",
                new SequentialCommandGroup(

                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BGrab1stSet"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BShoot1stSet"),

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW)),

                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "3BAltGrab2ndSet"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "2BAlt2Shoot"),

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW))),
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BGrab1stSet").getStartPose2d());

        // Grabs second ball and shoots, then grabs third and fourth balls and shoots
        this.add(
                "4 Ball",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BGrab1stSet"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BShoot1stSet"),
                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW)),
                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BGrab2ndSet"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),
                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BShoot2ndSet"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW))),
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BGrab1stSet").getStartPose2d());

        // Grabs second ball and shoots, then grabs third and a different fourth ball
        // and shoots
        this.add(
                "4 Ball Alt 1",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BGrab1stSet"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BShoot1stSet"),
                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW)),
                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BAlt1Grab2ndSet"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BAlt1Shoot2ndSet"),

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW))

                ),
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BGrab1stSet").getStartPose2d());

        // Shoots first ball, grabs second and third, then shoots
        this.add(
                "4 Ball Alt 2",
                new SequentialCommandGroup(

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW)),
                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "3BGrab"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "3BShoot"),

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW)),
                        new ParallelDeadlineGroup(
                                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BGrab2ndSet"),
                                new RunIntake(intakeSubsystem, loaderSubsystem)),

                        FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "4BShoot2ndSet"),

                        new ParallelDeadlineGroup(
                                new WaitCommand(SHOOT_TIME),
                                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem,
                                        ShooterConstants.SHOOTER_PROFILE_LOW))),
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, "3BGrab").getStartPose2d());

    }

    private void add(String name, SequentialCommandGroup cmd, Pose2d startingPose) {
        this.add(new NamedSequentialCommandGroup(
                new SequentialCommandGroup(
                        new InstantCommand(() -> driveSubsystem.resetOdometry(startingPose), driveSubsystem),
                        cmd),
                name));
    }
}
