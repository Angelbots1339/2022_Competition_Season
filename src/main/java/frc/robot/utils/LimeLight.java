// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableInstance;
import static frc.robot.Constants.LimelightConstants.*;

/** 
 * LimeLight Singleton Class
 * Use {@code LimeLight.getInstance} to get single instance
 */
public class LimeLight {
    private static LimeLight LIMELIGHT = null;


    //Getters 
    private static double getEntry(String key){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key).getDouble(0);
    }
    /**
     * @return Whether the limelight has any valid targets
     */
    public static boolean canSeeTarget(){
        return getEntry(entryType.VALID_TARGETS.toString()) == 0? false : true;
    }
    /**
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     */
    public static double getHorizontalOffset(){
        return getEntry(entryType.HORIZONTAL_OFFSET.toString());
    }
    /**
     * @return Vertical Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     */
    public static double getVerticalOffset(){
        return getEntry(entryType.VERTICAL_OFFSET.toString());
    }
    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public static double getTargetArea(){
        return getEntry(entryType.TARGET_AREA.toString());
    }
    /**
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public static double getSkew(){
        return getEntry(entryType.SKEW.toString());
    }
    /**
     * @return The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     */
    public static double getLatency(){
        return getEntry(entryType.LATENCY.toString());
    }
    /**
     * @return Sidelength of shortest side of the fitted bounding box (pixels)

     */
    public static double getShortestSideLength(){
        return getEntry(entryType.SHORTEST_SIDE.toString());
    }
    /**
     * @return Sidelength of longest side of the fitted bounding box (pixels)

     */
    public static double getLongestSideLength(){
        return getEntry(entryType.LONGEST_SIDE.toString());
    }
    /**
     * @return Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    public static double getHorizontalSideLength(){
        return getEntry(entryType.HORIZONTAL_BOUNDS.toString());
    }
    /**
     * @return Vertical sidelength of the rough bounding box (0 - 320 pixels)

     */
    public static double getVerticalSideLength(){
        return getEntry(entryType.VERTICAL_BOUNDS.toString());
    }
    /**
     * @return True active pipeline index of the camera (0 .. 9)
     */
    public static double getActivePipeline(){
        return getEntry(entryType.ACTIVE_PIPELINE.toString());
    }
    /**
     * @return Results of a 3D position solution, 6 numbers: Translation (x,y,y) Rotation(pitch,yaw,roll)
     */
    public static double[] getPose3D(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(entryType.POSE_3D.toString()).getDoubleArray(new double[6]);
    }

    //Setters
    private static void setEntry(String key, Number value){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(key).setNumber(value);
    }
    /**
     * Sets limelight’s LED state
     * @param  mode	
     <ul>
        <li>0 = use the LED Mode set in the current pipeline</li>
        <li>1 = force off</li>
        <li>2 = force blink</li>
        <li>3 = force on</li>
    </ul>
     */
    public static void setLEDMode(Integer mode){
        setEntry(entryType.LED_MODE.toString(), mode);
    }
    /**
     * Sets limelight’s operation mode
     * @param  mode	
     <ul>
        <li>0 = Vision processor</li>
        <li>1 = Driver Camera (Increases exposure, disables vision processing)</li>
    </ul>
     */
    public static void setCamMode(Integer mode){
        setEntry(entryType.CAM_MODE.toString(), mode);
    }
    /**
     * Sets limelight’s LED state
     * @param  pipeline	Select pipeline 0..9
     */
    public static void setPipeline(Integer pipeline){
        setEntry(entryType.PIPELINE.toString(), pipeline);
    }
    /**
     * Sets limelight’s streaming mode
     * @param  stream	
     <ul>
        <li>0 = Standard - Side-by-side streams if a webcam is attached to Limelight</li>
        <li>1 = PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream</li>
        <li>2 = PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream</li>
    </ul>
     */
    public static void setStream(Integer stream){
        setEntry(entryType.STREAM.toString(), stream);
    }
    /**
     * Allows users to take snapshots during a match
     * @param takeSnapshot
     <ul>
        <li>0 = Stop taking snapshots</li>
        <li>1 = Take two snapshots per second</li>
    </ul>
     */
    public static void snapshot(Integer takeSnapshot){
        setEntry(entryType.SNAPSHOT.toString(), takeSnapshot);
    }
    
    /**
     * @return The Single instance of Singleton LimeLight
     */
    public static LimeLight getInstance() {
        // To ensure only one instance is created
        if (LIMELIGHT == null) {
            LIMELIGHT = new LimeLight();
        }
        return LIMELIGHT;
    }
}
