package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class NamedSequentialCommandGroup extends SequentialCommandGroup{
    private String name;
    public NamedSequentialCommandGroup(SequentialCommandGroup sequentialCommandGroup, String name) {
        super(sequentialCommandGroup);
        this.name = name;
    }
    @Override
    public String toString() {
        return name;
    }

}
