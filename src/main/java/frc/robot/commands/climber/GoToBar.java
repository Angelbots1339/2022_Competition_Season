package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimbingSubsystem;

public class GoToBar extends SequentialCommandGroup{
    public GoToBar(ClimbingSubsystem climbingSubsystem, double angle, BooleanSupplier proceed) {
        addRequirements(climbingSubsystem);
        addCommands(
            new RotateToAngles(climbingSubsystem, 2),

            new WaitUntilCommand(proceed)
        );
    }
}
