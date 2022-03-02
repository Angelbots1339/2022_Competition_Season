package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    public AutoClimb(ClimbingSubsystem climbingSubsystem, BooleanSupplier proceed) {
        this.climbingSubsystem = climbingSubsystem;

        this.proceed = proceed;

        addRequirements(climbingSubsystem);
        addCommands(

            // First Bar Transfer
            new ArmsToSetpoints(24, climbingSubsystem),
            new ArmsToSetpoints(climbingSubsystem, 0.77, 24),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(17, climbingSubsystem),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.4),
            new WaitCommand(.25),
            new ArmsToSetpoints(climbingSubsystem, 0.01, 0),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.25, 0),
            new WaitUntilCommand(proceed),
            

            // Second Bar Transfer
            new ArmsToSetpoints(23.5, climbingSubsystem),
            new ArmsToSetpoints(climbingSubsystem, 0.77, 23.5),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(17, climbingSubsystem),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.4),
            new WaitCommand(.25),
            new ArmsToSetpoints(climbingSubsystem, 0.01, 0),
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
