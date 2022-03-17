package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.ShooterProfiles;

public class ShootTimed extends ParallelDeadlineGroup {
    public ShootTimed(IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem, ShooterSubsystem shooterSubsystem, ShooterProfiles shooterProfile, double seconds) {
        super(new WaitCommand(seconds),
            new Shoot(intakeSubsystem, loaderSubsystem, shooterSubsystem, shooterProfile));
    }
}
