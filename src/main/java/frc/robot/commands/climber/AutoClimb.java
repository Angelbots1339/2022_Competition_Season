package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
            new ArmsToSetpoints(climbingSubsystem, 0.69, 26),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.69, 8),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.4),
            new ArmsToSetpoints(climbingSubsystem, 0.02, 0),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.25, 0),
            new WaitUntilCommand(proceed),

            // Second Bar Transfer
            new ArmsToSetpoints(climbingSubsystem, 0.69, 26),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.69, 8),
            new WaitUntilCommand(proceed),
            new ArmsToSetpoints(climbingSubsystem, 0.4),
            new ArmsToSetpoints(climbingSubsystem, 0.02, 0),
            new WaitUntilCommand(proceed)
        );
    }

    // @Override
    // public void end(boolean interrupted) {
    //     CommandScheduler.getInstance().schedule(new ManualArms(climbingSubsystem, extendVolts, rotateVolts));
    //     CommandScheduler.getInstance().schedule(new RunCommand(() -> driveSubsystem.tankDriveVolts(0, 0), driveSubsystem));
    // }

}
