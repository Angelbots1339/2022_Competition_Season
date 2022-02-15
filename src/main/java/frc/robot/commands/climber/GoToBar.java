package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static frc.robot.Constants.ClimberConstants.*;
import frc.robot.subsystems.ClimbingSubsystem;


public class GoToBar extends SequentialCommandGroup{

    private ClimbingSubsystem climbingSubsystem;
    private ManualArms manualArms;

    @Override 
    public void initialize(){

        if(climbingSubsystem.getAutoClimbStarted()){

            CommandScheduler.getInstance().schedule(manualArms);

        } 
    }

/**
 * 
 * @param climbingSubsystem
 * @param angle
 * @param proceed 
 * @param emergencyStop
 */

    public GoToBar(ClimbingSubsystem climbingSubsystem, ManualArms manualArms, BooleanSupplier proceed) {

        this.climbingSubsystem = climbingSubsystem;
        this.manualArms = manualArms;
        
        addRequirements(climbingSubsystem);
        addCommands(

            // First Bar Transfer
            new AutoClimbMoveArms(climbingSubsystem, AUTO_EXTENTION_SETPOINT, AUTO_ROTATION_SETPOINT),
            new WaitUntilCommand(proceed),
            new AutoClimbMoveArms(climbingSubsystem, AUTO_EXTENTION_SETPOINT, ROTATOR_BACK_LIMIT_DEG),
            new WaitUntilCommand(proceed),
            new AutoClimbMoveArms(climbingSubsystem, EXTENDER_BOTTOM_LIMIT, ROTATOR_BACK_LIMIT_DEG),
            new WaitUntilCommand(proceed),

            // Second Bar Transfer
            new AutoClimbMoveArms(climbingSubsystem, AUTO_EXTENTION_SETPOINT, AUTO_ROTATION_SETPOINT),
            new WaitUntilCommand(proceed),
            new AutoClimbMoveArms(climbingSubsystem, AUTO_EXTENTION_SETPOINT, ROTATOR_BACK_LIMIT_DEG),
            new WaitUntilCommand(proceed),
            new AutoClimbMoveArms(climbingSubsystem, EXTENDER_BOTTOM_LIMIT, ROTATOR_BACK_LIMIT_DEG),
            new WaitUntilCommand(proceed)
        );
    }

}
