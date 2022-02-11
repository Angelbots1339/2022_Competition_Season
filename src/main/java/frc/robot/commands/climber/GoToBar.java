package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static frc.robot.Constants.ClimberConstants.*;
import frc.robot.subsystems.ClimbingSubsystem;

public class GoToBar extends SequentialCommandGroup{
    public GoToBar(ClimbingSubsystem climbingSubsystem, double angle, BooleanSupplier proceed) {
        addRequirements(climbingSubsystem);
        addCommands(
            // new MoveArms(climbingSubsystem, EXTENDER_TOP_LIMIT),
            // new WaitUntilCommand(proceed),
            // new MoveArms(climbingSubsystem, EXTENDER_BOTTOM_LIMIT),
            // new WaitUntilCommand(proceed),
            // new MoveArms(climbingSubsystem, EXTENDER_TOP_LIMIT, ROTATOR_FRONT_LIMIT_DEG),
            // new WaitUntilCommand(proceed),
            // new MoveArms(10, climbingSubsystem), //TODO change this to real value
            // new WaitUntilCommand(proceed),
            // new MoveArms(climbingSubsystem, EXTENDER_BOTTOM_LIMIT)
        );
    }

}
