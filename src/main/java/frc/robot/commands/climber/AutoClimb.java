package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.DriveSubsystem;


public class AutoClimb extends SequentialCommandGroup{

    BooleanSupplier proceed;
    ClimbingSubsystem climbingSubsystem;
    DriveSubsystem driveSubsystem;
    DoubleSupplier extendVolts, rotateVolts;

    /**
     * @param climbingSubsystem
     */
    public AutoClimb(ClimbingSubsystem climbingSubsystem, DriveSubsystem driveSubsystem, BooleanSupplier proceed, DoubleSupplier extendVolts, DoubleSupplier rotateVolts) {
        this.climbingSubsystem = climbingSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.proceed = proceed;
        this.rotateVolts = rotateVolts;
        this.extendVolts = extendVolts;

        addRequirements(climbingSubsystem);
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
            new ArmsToSetpoints(climbingSubsystem, 0.76, 24),
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
