package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
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
        intakeSubsystem.setDeployMotorsVolts(INTAKE_DEPLOY_VOLTS);
        intakeSubsystem.runIntake(INTAKE_DEPLOY_SPEED);
    }

    @Override
    public void execute() {

        intakeSubsystem.setDeployMotorsVolts(
            isLeftDeployed() ? 0 : INTAKE_DEPLOY_VOLTS,
            isRightDeployed() ? 0 : INTAKE_DEPLOY_VOLTS);
    }


    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.disable();
    }

    @Override
    public boolean isFinished() {
        return isLeftDeployed() && isRightDeployed();
    }

    private boolean isLeftDeployed() {
        return intakeSubsystem.getLeftPosition() > Constants.IntakeConstants.DEPLOY_SETPOINT;
    }

    private boolean isRightDeployed() {
        return intakeSubsystem.getRightPosition() > Constants.IntakeConstants.DEPLOY_SETPOINT;
    }
}
