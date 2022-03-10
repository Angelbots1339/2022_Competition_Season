package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Logging {
    private static ShuffleboardTab tab = Shuffleboard.getTab("RobotContainer");
    public static NetworkTableEntry log = tab.add("log", false).getEntry();
}
