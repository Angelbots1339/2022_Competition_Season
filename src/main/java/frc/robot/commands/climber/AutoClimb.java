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
import static frc.robot.Constants.ClimberConstants.*;


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
            new ArmsToSetpoints(24, climbingSubsystem), // Rotate arms back @ default speed
            new ArmsToSetpoints(climbingSubsystem, 0.77, 24), // Extend arms to high bar @ default speed
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(17, climbingSubsystem), // Rotate arms to smack high bar @ default speed
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.4), // Pull halfway up high bar @ default speed
            new WaitCommand(.25), // Brake mode stops arms from slamming into hard stops
            new ArmsToSetpoints(climbingSubsystem, 0.01, 0, SLOW_EXTENDER_VOLTS, MAX_ROTATOR_VOLTS), // Click hooks onto high bar @ slow speed (less swinging)
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.25, 0), // Drop high bar into hooks @ default speed
            new WaitUntilCommand(proceed),
            

            // Second Bar Transfer
            new ArmsToSetpoints(23.5, climbingSubsystem), // Rotate arms back @ default speed
            new ArmsToSetpoints(climbingSubsystem, 0.77, 23.5), // Extend arms to traverse bar @ default speed
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(17, climbingSubsystem), // Rotate arms to smack high bar @ default speed
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.4), // Pull halfway up traverse bar @ default speed
            new WaitCommand(.25), // Brake mode stops arms from slamming into hard stops
            new ArmsToSetpoints(climbingSubsystem, 0.01, 0, SLOW_EXTENDER_VOLTS, MAX_ROTATOR_VOLTS) // Click hooks onto traverse bar @ slow speed (less swinging)
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
