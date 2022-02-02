package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimbingSubsystem;

public class GoToBar extends SequentialCommandGroup{
    public GoToBar(ClimbingSubsystem climbingSubsystem, double angle, BooleanSupplier proceed) {
        addRequirements(climbingSubsystem);
        addCommands(
            new RotateClimberOut(climbingSubsystem),
            new WaitUntilCommand(proceed),
            new ExtendArms(climbingSubsystem, false),
            new WaitUntilCommand(proceed),
            new ExtendArms(climbingSubsystem, true),
            new WaitUntilCommand(proceed),
            new RotateClimberOut(climbingSubsystem)
            
        );
    }
}
