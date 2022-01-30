package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier fwd, rot;

    public ArcadeDrive(DoubleSupplier fwd, DoubleSupplier rot, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.fwd = fwd; this.rot = rot;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        driveSubsystem.tankDriveVolts(0, 0);
    }

    @Override
    public void execute() {
        super.execute();
        driveSubsystem.arcadeDrive(fwd, rot);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
