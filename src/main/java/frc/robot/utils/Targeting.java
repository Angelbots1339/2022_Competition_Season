package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Targeting {

    private static String CAMERA_NAME = "HP_Wide_Vision_HD_Camera";

    public static boolean isConnected() {
        return NetworkTablesHelper.getBoolean("CameraPublisher", new StringBuilder(CAMERA_NAME).append("-output").toString());
        // return NetworkTableInstance.getDefault()
        //     .getTable("CameraPublisher")
        //     .getEntry(new StringBuilder(CAMERA_NAME).append("-output").toString())
        //     .getBoolean(false);
    }

    public static boolean hasTarget() {
        return NetworkTablesHelper.getBoolean("photonvision", CAMERA_NAME, "hasTarget");
        // return NetworkTableInstance.getDefault()
        //     .getTable("photonvision")
        //     .getSubTable(CAMERA_NAME)
        //     .getEntry("hasTarget")
        //     .getBoolean(false);
    }

    public static double getTargetXOffset() {
        return NetworkTablesHelper.getDouble("photonvision", CAMERA_NAME, "targetPixelsX");
    }

    public static boolean setPipeline(double pipelineIndex) {
        return NetworkTableInstance.getDefault().getTable("photovision").getSubTable(CAMERA_NAME).getEntry("pipelineIndex").setDouble(pipelineIndex);
    }




}
