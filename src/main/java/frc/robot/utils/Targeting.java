package frc.robot.utils;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Targeting {

    private static double[] recentFrames = { 0, 0, 0, };
    private static String CAMERA_NAME = "rPi Camera 0";
    private static int index = 0;

    public static boolean isConnected() {
        return NetworkTablesHelper.getBoolean("CameraPublisher",
                new StringBuilder(CAMERA_NAME).append("-output").toString(), "connected");
    }

    public static boolean hasTarget() {
        return NetworkTablesHelper.getBoolean("photonvision", CAMERA_NAME, "hasTarget");
    }

    /**
     * Discards inputs that are in the upper half of the screen.
     * @return Median of the results of last 5 frames
     */
    public static double getTargetXOffset() {
        if(hasTarget()){
            recentFrames[index] = (160 - NetworkTablesHelper.getDouble("photonvision", CAMERA_NAME, "targetPixelsX"));
        } else{
            recentFrames[index] = 0;
        }

        double[] sortedFrames = recentFrames;
        Arrays.sort(sortedFrames);
        index++;
        index %= recentFrames.length;
        return sortedFrames[1];

        
    }

    public static boolean setPipeline(double pipelineIndex) {
        return NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(CAMERA_NAME)
                .getEntry("pipelineIndex").setDouble(pipelineIndex);
    }
    
    public static double getPipeline() {
        return NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(CAMERA_NAME)
                .getEntry("pipelineIndex").getDouble(-1);
    }

}
