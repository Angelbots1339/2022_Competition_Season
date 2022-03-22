// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.AutoConstants.*;

import frc.robot.commands.Intake.EjectBalls;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.NamedSequentialCommandGroup;
import frc.robot.utils.ShooterProfiles;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public final class AutoSequences extends ArrayList<NamedSequentialCommandGroup> {

    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LoaderSubsystem loaderSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    public AutoSequences(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem,
            LoaderSubsystem loaderSubsystem,
            ShooterSubsystem shooterSubsystem) {
        super();
        this.driveSubsystem = driveSubsystem;
        this.loaderSubsystem = loaderSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        // Example auto path
        // Turns on intake
        // Drives 1 meter
        // Turns off intake
        // Shoots low for 2 seconds
        this.add(
                "Example",
                "2Meter",
                new SequentialCommandGroup(
                        grabTimed("2Meter", 2),
                        shoot(SHOOT_TIME_2B, SHOOTER_PROFILE_LOW)
                ));

        this.add(
                "Zero Move",
                "2Meter",
                new SequentialCommandGroup(grab("2Meter"))

        );

        this.add(
                "Zero no move",
                "2Meter",
                new SequentialCommandGroup(new InstantCommand())
        );
        

        // Shoots ball and drives past line
        this.add(
                "1 Ball",
                "1BForward",
                new SequentialCommandGroup(
                        shoot(SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH),
                        drive("1BForward")
                ));

        // Shoots ball and hides opponent's
        this.add(
                "1 Ball Alt 1", 
                "1BAlt1Hide", 
                new SequentialCommandGroup(
                        shoot(SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH),
                        grab("1BAlt1Hide"),
                        eject(3)
                ));

        // Grabs second ball and shoots
        this.add(
                "2 Ball",
                "2BGrab",
                new SequentialCommandGroup(
                                grabDriveShoot("2BGrab", "2BShoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH)
                                // ,
                                // grab("2BHide"),
                                // eject(3)
                        )
                
                );
        this.add(
                "2 Ball Hide",
                "2BGrab",
                new SequentialCommandGroup(
                                grabDriveShoot("2BGrab", "2BShoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH)
                                ,
                                grab("2BHide"),
                                eject(3),
                                new TurnSimple(driveSubsystem, .5, 3)
                        )
                
                );

        // Grabs a different second ball and shoots
        this.add(
                "2 Ball Alt 1",
                "4BGrab1stSet",
                grabDriveShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH)
                );
        // Grabs another different second ball and shoots
        this.add(
                "2 Ball Alt 2",
                "2BAlt2Grab",
                grabDriveShoot("2BAlt2Grab", "2BAlt2Shoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH)
                );

        // Shoots first ball, grabs second and third, then shoots
        this.add(
                "3 Ball",
                "3BGrab",
                new SequentialCommandGroup(
                        shoot(SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH),
                        grabDriveShoot("3BGrab", "3BShoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH)
                ));

        // Grabs first ball, shoots, grabs 2nd ball, shoots
        this.add(
                "3 Ball Alt 1",
                "4BGrab1stSet",
                new SequentialCommandGroup(
                        grabDriveShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH),
                        grabDriveShoot("3BAltGrab2ndSet", "2BAlt2Shoot", SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH)
                ));

        // Grabs second ball and shoots, then grabs third and fourth balls and shoots
        this.add(
                "4 Ball Blue",
                "4BGrab1stSet",
                new SequentialCommandGroup(
                        grabGrabShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, 2),
                        grabGrabShoot("4BGrab2ndSet", "4BShoot2ndSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, 2)
                ));
        // Grabs second ball and shoots, then grabs third and fourth balls and shoots
        this.add(
                "4 Ball Red",
                "4BGrab1stSet",
                new SequentialCommandGroup(
                        grabDriveShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH),
                        grabGrabShoot("4BGrab2ndSetRed", "4BShoot2ndSetRed", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, 2)
                ));

        // Grabs second ball and shoots, then grabs third and a different fourth ball
        // and shoots
        this.add(
                "4 Ball Alt 1",
                "4BGrab1stSet",
                new SequentialCommandGroup(
                        grabDriveShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH),
                        grabDriveShoot("4BAlt1Grab2ndSet", "4BAlt1Shoot2ndSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH)
                ));

        
        // Grabs second ball, shoots, grabs far ball + human ball, shoots
        this.add(
                "4 Ball Alt 3",
                "2BGrab",
                new SequentialCommandGroup(
                        grabDriveShoot("2BGrab", "2BShoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH),
                        grabDriveShoot("4BAlt3Grab", "4BAlt3Shoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH)
                ));
        // Shoots first ball, grabs second and third, then shoots, grabs third and fourth, shoots
        this.add(
                "5 Ball",
                "3BGrab",
                new SequentialCommandGroup(
                        shoot(SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH),
                        grabDriveShoot("3BGrab", "3BShoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH),
                        grabDriveShoot("4BGrab2ndSet", "4BShoot2ndSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH)
                ));

        this.add(
                "Turn To 0 Degrees",
                "3BGrab",
                new SequentialCommandGroup(turnToAngle(0))
                
                );
    }

    private void add(String name, String firstPath, SequentialCommandGroup cmd) {
        
        this.add(new NamedSequentialCommandGroup(
                new SequentialCommandGroup(
                        new InstantCommand(() -> driveSubsystem.resetOdometry(getStartPose(firstPath)), driveSubsystem),
                        cmd),
                name));

       
    }

   

    private TurnToAngle turnToAngle(double angle) {

        return new TurnToAngle(driveSubsystem, angle);
    }

    private ParallelDeadlineGroup grab(String pathName) {
        return new ParallelDeadlineGroup(
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, pathName),
                new RunIntake(intakeSubsystem, loaderSubsystem), new IdleShooter(shooterSubsystem));
    }

    private ParallelDeadlineGroup grabTimed(String pathName, double grabTime) {
        return new ParallelDeadlineGroup(
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, pathName),
                new ParallelDeadlineGroup(
                        new WaitCommand(grabTime), new RunIntake(intakeSubsystem, loaderSubsystem)),
                new IdleShooter(shooterSubsystem));
    }

    private Command drive(String pathName) {
        return new ParallelDeadlineGroup(FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, pathName), new IdleShooter(shooterSubsystem));
    }

    private ParallelDeadlineGroup shoot(double time, ShooterProfiles shooterProfile) {
        return new ParallelDeadlineGroup(
                new WaitCommand(time),
                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem, shooterProfile));
    }

    private ParallelDeadlineGroup eject(double time) {
        return new ParallelDeadlineGroup(
                new WaitCommand(time), 
                new EjectBalls(intakeSubsystem, loaderSubsystem));
    }

    private SequentialCommandGroup grabDriveShoot(String grabPath, String drivePath, double time, ShooterProfiles shooterProfile) {
        return new SequentialCommandGroup(
                grab(grabPath), drive(drivePath), shoot(time, shooterProfile)
        );
    }
    private SequentialCommandGroup grabGrabShoot(String grabPath, String drivePath, double shootTime, ShooterProfiles shooterProfile, double grabTime) {
        return new SequentialCommandGroup(
                grab(grabPath), grabTimed(drivePath, grabTime), shoot(shootTime, shooterProfile)
        );
    }

    private Pose2d getStartPose(String pathName) {
        return FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, pathName).getStartPose2d();
    }
}
