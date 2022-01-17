package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier fwd, rot, zero;

    public ArcadeDrive(DoubleSupplier fwd, DoubleSupplier rot, DriveSubsystem m_driveSubsystem) {
        this.m_driveSubsystem = m_driveSubsystem;
        this.fwd = fwd; this.rot = rot;
        zero = () -> 0;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        m_driveSubsystem.resetOdometry(new Pose2d());
        m_driveSubsystem.arcadeDrive(zero, zero);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        m_driveSubsystem.arcadeDrive(fwd, rot);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
