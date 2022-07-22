// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.AutoConstants.*;

import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.TurnSimple;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.EjectBalls;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.IdleShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.NamedSequentialCommandGroup;
import frc.robot.utils.ShooterProfiles;

/**
 * Adds each auto sequence to an array list to easily populate shuffleboard
 */
public final class AutoSequences extends ArrayList<NamedSequentialCommandGroup> {

    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LoaderSubsystem loaderSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final boolean reject;
    private static final String FAST = "fast/output/";
    private static final String REGULAR = "regular/output/";

    public AutoSequences(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem,
            LoaderSubsystem loaderSubsystem,
            ShooterSubsystem shooterSubsystem,
            boolean reject) {
        super();
        this.driveSubsystem = driveSubsystem;
        this.loaderSubsystem = loaderSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.reject = reject;

        this.add(
                "Exmaple",
                "2Meter",
                new SequentialCommandGroup(
                        grabTimed("2Meter", 2, REGULAR),
                        shoot(SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH),
                        drive("2Meter", REGULAR)
                ));

        this.add(
                "Taxi",
                "2Meter",
                new SequentialCommandGroup(grab("2Meter", REGULAR))

        );

        this.add(
                "Wait",
                "2Meter",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                idleDrive(), 
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2)
                )))
        );       

        // Shoots ball and drives past line
        this.add(
                "1 Ball",
                "1BForward",
                new SequentialCommandGroup(
                        shoot(SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH),
                        grabTimed("1BForward", 2, REGULAR)
                ));

        this.add(
                "1 Ball Left",
                "1BForwardLeft",
                new SequentialCommandGroup(
                        shoot(SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH),
                        grabTimed("1BForwardLeft", 2, REGULAR)
                ));

        // Shoots ball and hides opponent's
        this.add(
                "1 Ball Alt 1", 
                "1BAlt1Hide", 
                new SequentialCommandGroup(
                        shoot(SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH),
                        grab("1BAlt1Hide", REGULAR),
                        eject(3)
                ));

        // Grabs second ball and shoots
        this.add(
                "2 Ball",
                "2BGrab",
                new SequentialCommandGroup(
                                grabDriveShoot("2BGrab", "2BShoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, REGULAR)
                        )
                
                );
                
        this.add(
                "2 Ball Hide",
                "2BGrab",
                new SequentialCommandGroup(
                                grabDriveShoot("2BGrab", "2BShoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, REGULAR),
                                grab("2BHide", REGULAR),
                                eject(3),
                                turnTimed(HALF_TURN_TIME, TURN_VOLTS)
                        )
                
                );

        this.add(
                "2 Ball Hide Wait 4 Fender",
                "2BGrab",
                new SequentialCommandGroup(
                                grabDriveShootWait("2BGrab", "2BShoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, 2, REGULAR),
                                grab("2BHide", REGULAR),
                                eject(3),
                                turnTimed(HALF_TURN_TIME, TURN_VOLTS)
                        )
                
                );

        // Grabs a different second ball and shoots
        this.add(
                "2 Ball Alt 1",
                "4BGrab1stSet",
                grabGrabShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B + 5, SHOOTER_PROFILE_HIGH, 2, REGULAR)
                );
        // Grabs another different second ball and shoots
        this.add(
                "2 Ball Alt 2",
                "2BAlt2Grab",
                grabDriveShoot("2BAlt2Grab", "2BAlt2Shoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, REGULAR)
                );

        // Shoots first ball, grabs second and third, then shoots
        this.add(
                "3 Ball",
                "3BGrab",
                new SequentialCommandGroup(
                        shoot(SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH),
                        grabDriveShoot("3BGrab", "3BShoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, REGULAR)
                ));

        // Grabs first ball, shoots, grabs 2nd ball, shoots
        this.add(
                "3 Ball Alt 1",
                "4BGrab1stSet",
                new SequentialCommandGroup(
                        grabDriveShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, REGULAR),
                        grabDriveShoot("3BAltGrab2ndSet", "2BAlt2Shoot", SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH, REGULAR)
                ));

        // Grabs second ball and shoots, then grabs third and fourth balls and shoots
        this.add(
                "4 Ball Blue",
                "4BGrab1stSet",
                new SequentialCommandGroup(
                        grabGrabShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, 2, REGULAR),
                        grabGrabShoot("4BGrab2ndSetBlue", "4BShoot2ndSetBlue", SHOOT_TIME_2B + 5, SHOOTER_PROFILE_HIGH, 2, REGULAR)
                ));
        // Grabs second ball and shoots, then grabs third and fourth balls and shoots
        this.add(
                "4 Ball Red",
                "4BGrab1stSet",
                new SequentialCommandGroup(
                        grabGrabShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, 2, REGULAR),
                        grabGrabShoot("4BGrab2ndSetRed", "4BShoot2ndSetRed", SHOOT_TIME_2B + 5, SHOOTER_PROFILE_HIGH, 2, REGULAR)
                ));

        // Grabs second ball and shoots, then grabs third and a different fourth ball
        // and shoots
        this.add(
                "4 Ball Alt 1",
                "4BGrab1stSet",
                new SequentialCommandGroup(
                        grabDriveShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, REGULAR),
                        grabDriveShoot("4BAlt1Grab2ndSet", "4BAlt1Shoot2ndSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, REGULAR)
                ));

        
        // Grabs second ball, shoots, grabs far ball + human ball, shoots
        this.add(
                "4 Ball Alt 3",
                "2BGrab",
                new SequentialCommandGroup(
                        grabDriveShoot("2BGrab", "2BShoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, REGULAR),
                        grabDriveShoot("4BAlt3Grab", "4BAlt3Shoot", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, REGULAR)
                ));
        // Grabs 2nd ball, shoots 2, grabs 3rd, shoots 1, grabs 4th and 5th, shoots 2
        this.add(
                "5 Ball",
                "4BGrab1stSet",
                new SequentialCommandGroup(
                        grabGrabShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, 2, FAST),
                        grabGrabShoot("5BGrab2nd", "5BShoot2nd", SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH, 2, FAST),
                        grabGrabShoot("5BGrab3rdSet", "5BShoot3rdSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, 2, FAST)

                ));
        // Grabs 2nd ball, shoots 2, grabs 3rd, shoots 1, grabs 4th & 5th, shoots 2
        this.add(
                "5 Ball Red",
                "4BGrab1stSet",
                new SequentialCommandGroup(
                        grabDriveShoot("4BGrab1stSet", "4BShoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, FAST),
                        grabDriveShoot("5BGrab2ndRed", "5BShoot2ndRed", SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH, FAST),
                        grabDriveShoot("5BGrab3rdSetRed", "5BShoot3rdSetRed", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, FAST)

                ));
        // Shoots 1, grabs 2nd, turns to 3rd, grabs 3rd, shoots 2, grabs 4th & 5th, shoots 2
        this.add(
                "5 Ball Alt 1",
                "5BAlt1Grab1stSet1",
                new SequentialCommandGroup(
                        shoot(SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH),
                        grab("5BAlt1Grab1stSet1", FAST),
                        turnToAngle(angleFromPath("5BAlt1Grab1stSet2", FAST)),
                        grabDriveShoot("5BAlt1Grab1stSet2", "5BAlt1Shoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, FAST),
                        grabDriveShoot("5BAlt1Grab2ndSet", "5BAlt1Shoot2ndSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, FAST)
                ));
        // Shoots 1, grabs 2nd, turns to 3rd, grabs 3rd, shoots 2, grabs 4th & 5th, shoots 2
        this.add(
                "5 Ball Alt 1 Red",
                "5BAlt1Grab1stSet1",
                new SequentialCommandGroup(
                        shoot(SHOOT_TIME_1B, SHOOTER_PROFILE_HIGH),
                        grab("5BAlt1Grab1stSet1", FAST),
                        turnToAngle(angleFromPath("5BAlt1Grab1stSet2", FAST)),
                        grabDriveShoot("5BAlt1Grab1stSet2", "5BAlt1Shoot1stSet", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, FAST),
                        grabDriveShoot("5BAlt1Grab2ndSetRed", "5BAlt1Shoot2ndSetRed", SHOOT_TIME_2B, SHOOTER_PROFILE_HIGH, FAST)
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
                        new InstantCommand(() -> driveSubsystem.resetOdometry(getStartPose(firstPath, REGULAR)), driveSubsystem),
                        cmd,
                        idleDrive()),
                name));

       
    }

    /**
     * Feed drive watchdog to not generate errors
     * @return
     */
    private RunCommand idleDrive() {
        return new RunCommand(() -> driveSubsystem.disable(), driveSubsystem);
    }
    
    private TurnToAngle turnToAngle(double angle) {
        return new TurnToAngle(driveSubsystem, angle);
    }

    private TurnSimple turnTimed(double time, double volts) {
        return new TurnSimple(driveSubsystem, time, volts);
    }

    /**
     * Drive a path while intake is running
     * @param pathName
     * @return
     */
    private ParallelDeadlineGroup grab(String pathName, String folderName) {
        return new ParallelDeadlineGroup(
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, pathName, folderName),
                new SequentialCommandGroup(
                        new DeployIntake(intakeSubsystem)
                        .andThen(new RunIntake(intakeSubsystem, loaderSubsystem))
                ),
                new IdleShooter(shooterSubsystem));
    }

    /**
     * Drive a path, run intake on a timer
     * @param pathName
     * @param grabTime
     * @return Built command
     */
    private ParallelDeadlineGroup grabTimed(String pathName, double grabTime, String folderName) {
        return new ParallelDeadlineGroup(
                FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, pathName, folderName),
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(grabTime),
                                new DeployIntake(intakeSubsystem)
                                .andThen(new RunIntake(intakeSubsystem, loaderSubsystem))  
                ),
                
                new IdleShooter(shooterSubsystem)));
    }

    private Command drive(String pathName, String folderName) {
        return new ParallelDeadlineGroup(FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, pathName, folderName), new IdleShooter(shooterSubsystem));
    }

    /**
     * Shoot for a given time
     * @param time
     * @param shooterProfile
     * @return Built command
     */
    private ParallelDeadlineGroup shoot(double time, ShooterProfiles shooterProfile) {
        return new ParallelDeadlineGroup(
                new WaitCommand(time),
                new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem, shooterProfile, reject),
                idleDrive());
    }

    /**
     * Run intake backwards to spit ball
     * @param time
     * @return
     */
    private ParallelDeadlineGroup eject(double time) {
        return new ParallelDeadlineGroup(
                new WaitCommand(time), 
                new EjectBalls(intakeSubsystem, loaderSubsystem),
                idleDrive());
    }

    private SequentialCommandGroup grabDriveShoot(String grabPath, String drivePath, double time, ShooterProfiles shooterProfile, String folderName) {
        return new SequentialCommandGroup(
                grab(grabPath, folderName), drive(drivePath, folderName), shoot(time, shooterProfile)
        );
    }

    private SequentialCommandGroup grabDriveShootWait(String grabPath, String drivePath, double time, ShooterProfiles shooterProfile, double seconds, String folderName) {
        return new SequentialCommandGroup(
                grab(grabPath, folderName), new WaitCommand(seconds), drive(drivePath, folderName), shoot(time, shooterProfile)
        );
    }

    private SequentialCommandGroup grabGrabShoot(String grabPath, String drivePath, double shootTime, ShooterProfiles shooterProfile, double grabTime, String folderName) {
        return new SequentialCommandGroup(
                grab(grabPath, folderName), grabTimed(drivePath, grabTime, folderName), shoot(shootTime, shooterProfile)
        );
    }

    private Pose2d getStartPose(String pathName, String folderName) {
        return FollowTrajectory.followTrajectoryFromJSON(driveSubsystem, pathName, folderName).getStartPose2d();
    }

    private double angleFromPath(String pathName, String folderName) {
        return getStartPose(pathName, folderName).getRotation().getDegrees();
    }
}
