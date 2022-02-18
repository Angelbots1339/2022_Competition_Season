package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static frc.robot.Constants.ClimberConstants.*;
import frc.robot.subsystems.ClimbingSubsystem;


public class AutoClimb extends SequentialCommandGroup{

    /**
     * @param climbingSubsystem
     */
    public AutoClimb(ClimbingSubsystem climbingSubsystem) {
        addRequirements(climbingSubsystem);
        addCommands(

            // First Bar Transfer
            new ArmsToSetpoints(climbingSubsystem, AUTO_EXTENSION_SETPOINT, AUTO_ROTATION_SETPOINT),
            new ArmsToSetpoints(climbingSubsystem, AUTO_EXTENSION_SETPOINT, ROTATOR_BACK_LIMIT_DEG),
            new ArmsToSetpoints(climbingSubsystem, EXTENDER_BOTTOM_LIMIT, ROTATOR_BACK_LIMIT_DEG),

            // Second Bar Transfer
            new ArmsToSetpoints(climbingSubsystem, AUTO_EXTENSION_SETPOINT, AUTO_ROTATION_SETPOINT),
            new ArmsToSetpoints(climbingSubsystem, AUTO_EXTENSION_SETPOINT, ROTATOR_BACK_LIMIT_DEG),
            new ArmsToSetpoints(climbingSubsystem, EXTENDER_BOTTOM_LIMIT, ROTATOR_BACK_LIMIT_DEG)
        );
    }

}
