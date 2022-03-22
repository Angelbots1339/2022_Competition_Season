package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LoaderConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.ShooterProfiles;

public class ShootTimed extends CommandBase {
    Timer timer;
    IntakeSubsystem intakeSubsystem;
    LoaderSubsystem loaderSubsystem;
    ShooterSubsystem shooterSubsystem;
    double seconds;

    public ShootTimed(IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem,
            ShooterSubsystem shooterSubsystem, ShooterProfiles shooterProfile, double seconds) {
                timer = new Timer();
                this.intakeSubsystem = intakeSubsystem;
                this.shooterSubsystem = shooterSubsystem;
                this.loaderSubsystem = loaderSubsystem;
                this.seconds = seconds;
    }

    @Override
    public void initialize() {

    timer.start();    

    }


    @Override
    public void execute() {

        shooterSubsystem.setAimWheelRPM(ShooterConstants.SHOOTER_PROFILE_REJECT.getAimRPM());
        shooterSubsystem.setPowerWheelRPM(ShooterConstants.SHOOTER_PROFILE_REJECT.getPowerRPM());

        loaderSubsystem.runLoader(LoaderConstants.MAX_LOADER_SPEED);

        intakeSubsystem.runIndexerLow(IntakeConstants.MAX_INDEXER_PERCENT);
    }

    @Override
    public void end(boolean interrupt){
        shooterSubsystem.setAimWheelRPM(0);
        shooterSubsystem.setPowerWheelRPM(0);

        loaderSubsystem.runLoader(0);

        intakeSubsystem.runIndexerLow(0);


    }

    @Override 
    public boolean isFinished(){

        return timer.get() >= seconds;
    }


}
