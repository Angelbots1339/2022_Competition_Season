package frc.robot.utils;

import com.revrobotics.ColorSensorV3.RawColor;

public class ColorRange {
    public RawColor color;
    private double errorThreshold;

    public ColorRange(RawColor color, double errorThreshold) {
        this.color = color;
        this.errorThreshold = errorThreshold;
    }

    public ColorRange(int red, int green, int blue, double errorThreshold) {
        this(new RawColor(red, green, blue, 0), errorThreshold);
    }

    public double getColorError(RawColor other) {
        return 
            Math.abs(color.red - other.red) +
            Math.abs(color.green - other.green) +
            Math.abs(color.blue - other.blue);
    }

    public boolean colorMatch(RawColor other) {
        return getColorError(other) < errorThreshold;
    }
}
