package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.utils.Candle;
import frc.robot.utils.Candle.LEDState;

import static frc.robot.Constants.ClimberConstants.*;


public class AutoClimb extends SequentialCommandGroup{
   


    public AutoClimb(ClimbingSubsystem climbingSubsystem, BooleanSupplier proceed) {

        addRequirements(climbingSubsystem);
        addCommands(
            new InstantCommand(() -> {Candle.getInstance().changeLedState(LEDState.Climbing);}),

            // First Bar Transfer
            // Rotate arms back @ default speed
            new ArmsToSetpoints(25, climbingSubsystem), // 25
            // Extend arms to high bar @ default speed
            new ArmsToSetpoints(climbingSubsystem, 0.77, 25, MAX_EXTENDER_VOLTS, 3), 
            new WaitUntilCommand(proceed),
            // Rotate arms to smack high bar @ default speed
            new ArmsToSetpoints(17, climbingSubsystem), 
            new WaitUntilCommand(proceed),
            // Slip hooks off mid bar, stall at 9 degrees
            //new ArmsToSetpoints(climbingSubsystem, 0.57, 9, DROP_EXTENDER_VOLTS, 3.5, true, false),
            // Brake mode stops arms from slamming
            new ParallelDeadlineGroup(
                // Wait while stalling at 9 degrees
                new WaitCommand(1),
                new ArmsToSetpoints(climbingSubsystem, 0.45, 9, DROP_EXTENDER_VOLTS, 3.5, true, true)
            ), 
            // Click hooks onto high bar @ default speed
            new ArmsToSetpoints(climbingSubsystem, 0.45, -0.5, DROP_EXTENDER_VOLTS, MAX_ROTATOR_VOLTS), 
            // new WaitCommand(.1),
            // Click hooks onto high bar @ default speed
            new ArmsToSetpoints(climbingSubsystem, -0.01, -0.5, PULLUP_VOLTS, MAX_ROTATOR_VOLTS), //-0.01 length setpoint
            new WaitUntilCommand(proceed),
            // Drop high bar into hooks @ slow speed
            new ArmsToSetpoints(climbingSubsystem, 0.25, 0, DROP_EXTENDER_VOLTS, MAX_ROTATOR_VOLTS), 
            new WaitUntilCommand(proceed),
            

            // Second Bar Transfer
            // Rotate arms back @ default speed
            new ArmsToSetpoints(24, climbingSubsystem), 
            // Extend arms almost to traverse bar @ default speed
            new ArmsToSetpoints(climbingSubsystem, 0.5, 24,  MAX_EXTENDER_VOLTS, 3), 
            new WaitUntilCommand(proceed), 
            // Finish extension to get bar traverse bar @ default speed
            new ArmsToSetpoints(climbingSubsystem, 0.77, 23.5),
            new WaitUntilCommand(proceed),
            // Rotate arms to smack high bar @ default speed
            new ArmsToSetpoints(17, climbingSubsystem), 
            new WaitUntilCommand(proceed),
            // Pull halfway up traverse bar @ slow speed
            // new ArmsToSetpoints(climbingSubsystem, 0.3, DROP_EXTENDER_VOLTS, MAX_ROTATOR_VOLTS), 
            // Brake mode stops arms from slamming into hard stops
            new ParallelDeadlineGroup(
                // Wait while stalling at 9 degrees
                new WaitCommand(1),
                new ArmsToSetpoints(climbingSubsystem, 0.45, 9, DROP_EXTENDER_VOLTS, 3.5, true, true)
            ), 
            // // Click hooks onto traverse bar @ default speed
            // new ArmsToSetpoints(climbingSubsystem, 0.3, 6, DROP_EXTENDER_VOLTS, MAX_ROTATOR_VOLTS), 
            // Click hooks onto traverse bar @ default speed
            new ArmsToSetpoints(climbingSubsystem, 0.00, 0, PULLUP_VOLTS, MAX_ROTATOR_VOLTS)  // 0.00 length
        );
    }
}




// --- OLD CLIMB ---
// package frc.robot.commands.climber;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.subsystems.ClimbingSubsystem;
// import static frc.robot.Constants.ClimberConstants.*;

// /**
//  * Breaks a climb down into multiple setpoints for the arms, 
//  * with the driver advancing each stage
//  */
// public class AutoClimb extends SequentialCommandGroup{
   

//     /**
//      * @param climbingSubsystem
//      * @param proceed Driver input for advancing to the next stage
//      */
//     public AutoClimb(ClimbingSubsystem climbingSubsystem, BooleanSupplier proceed) {

//         addRequirements(climbingSubsystem);
//         addCommands(

//             // First Bar Transfer
//             // Rotate arms back @ default speed
//             new ArmsToSetpoints(25, climbingSubsystem), // 25
//             // Extend arms to high bar @ default speed
//             new ArmsToSetpoints(climbingSubsystem, 0.77, 25, MAX_EXTENDER_VOLTS, 3), 
//             new WaitUntilCommand(proceed),
//             // Rotate arms to smack high bar @ default speed
//             new ArmsToSetpoints(17, climbingSubsystem), 
//             new WaitUntilCommand(proceed),
//             // Pull halfway up high bar @ slow speed
//             new ArmsToSetpoints(climbingSubsystem, 0.35, DROP_EXTENDER_VOLTS, MAX_ROTATOR_VOLTS), 
//             // Brake mode stops arms from slamming into hard stops
//             new WaitCommand(.5), 
//             // Click hooks onto high bar @ default speed
//             new ArmsToSetpoints(climbingSubsystem, 0.35, 9, DROP_EXTENDER_VOLTS, MAX_ROTATOR_VOLTS), 
//             new WaitCommand(.1),
//             // Click hooks onto high bar @ default speed
//             new ArmsToSetpoints(climbingSubsystem, -0.01, -0.5,PULLUP_VOLTS, MAX_ROTATOR_VOLTS),
//             new WaitUntilCommand(proceed),
//             // Drop high bar into hooks @ slow speed
//             new ArmsToSetpoints(climbingSubsystem, 0.25, 0, DROP_EXTENDER_VOLTS, MAX_ROTATOR_VOLTS), 
//             new WaitUntilCommand(proceed),
            

//             // Second Bar Transfer
//             // Rotate arms back @ default speed
//             new ArmsToSetpoints(24, climbingSubsystem), 
//             // Extend arms almost to traverse bar @ default speed
//             new ArmsToSetpoints(climbingSubsystem, 0.5, 24,  MAX_EXTENDER_VOLTS, 3), 
//             new WaitUntilCommand(proceed), 
//             // Finish extension to get bar traverse bar @ default speed
//             new ArmsToSetpoints(climbingSubsystem, 0.77, 23.5),
//             new WaitUntilCommand(proceed),
//             // Rotate arms to smack high bar @ default speed
//             new ArmsToSetpoints(17, climbingSubsystem), 
//             new WaitUntilCommand(proceed),
//             // Pull halfway up traverse bar @ slow speed
//             new ArmsToSetpoints(climbingSubsystem, 0.3, DROP_EXTENDER_VOLTS, MAX_ROTATOR_VOLTS), 
//             // Brake mode stops arms from slamming into hard stops
//             new WaitCommand(.25), 
//             // Click hooks onto traverse bar @ default speed
//             new ArmsToSetpoints(climbingSubsystem, 0.3, 6, DROP_EXTENDER_VOLTS, MAX_ROTATOR_VOLTS), 
//             // Click hooks onto traverse bar @ default speed
//             new ArmsToSetpoints(climbingSubsystem, 0.00, 0, PULLUP_VOLTS, MAX_ROTATOR_VOLTS) 
//             // 
//         );
//     }
// }
