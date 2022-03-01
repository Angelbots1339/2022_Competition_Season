package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoClimb extends SequentialCommandGroup{

    BooleanSupplier proceed;
    ClimbingSubsystem climbingSubsystem;
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    LoaderSubsystem loaderSubsystem;
    ShooterSubsystem shooterSubsystem;
    DoubleSupplier extendVolts, rotateVolts;

    /**
     * @param climbingSubsystem
     */
    public AutoClimb(ClimbingSubsystem climbingSubsystem, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem, ShooterSubsystem shooterSubsystem, BooleanSupplier proceed, DoubleSupplier extendVolts, DoubleSupplier rotateVolts) {
        this.climbingSubsystem = climbingSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.loaderSubsystem = loaderSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        this.proceed = proceed;
        this.rotateVolts = rotateVolts;
        this.extendVolts = extendVolts;

        addRequirements(climbingSubsystem, driveSubsystem, intakeSubsystem, loaderSubsystem, shooterSubsystem);
        addCommands(

            // First Bar Transfer
            new ArmsToSetpoints(climbingSubsystem, 0.76, 24),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.76, 17),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.4),
            new ArmsToSetpoints(climbingSubsystem, 0.03, 0),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.25, 0),
            new WaitUntilCommand(proceed),
            

            // Second Bar Transfer
            new ArmsToSetpoints(climbingSubsystem, 0.75, 23.5),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.76, 17),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.4),
            new ArmsToSetpoints(climbingSubsystem, 0.03, 0),
            new WaitUntilCommand(proceed)
        );
    }

    // @Override
    // public void end(boolean interrupted) {
        

    // }
    
    // @Override
    // public boolean isFinished() {
    //   return climbingSubsystem.areMotorsStalling();
    // }

}
