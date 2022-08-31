package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import static frc.robot.Constants.IntakeConstants.*;

public class DeployIntake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public DeployIntake(IntakeSubsystem intakeSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSubsystem);
        this.intakeSubsystem = intakeSubsystem;

    }

    @Override
    public void initialize() {
        intakeSubsystem.runIntake(INTAKE_DEPLOY_SPEED);
        intakeSubsystem.runIndexerLow(MAX_INDEXER_PERCENT);

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.disable();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
