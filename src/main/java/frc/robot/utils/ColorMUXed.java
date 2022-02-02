package frc.robot.utils;

import com.revrobotics.ColorSensorV3;
import frc.robot.utils.Multiplexer.MultiplexerSwitchException;
import static frc.robot.Constants.*;

/**
 * Wrapper class for ColorSensorV3 to use {@link Multiplexer}
 */
public class ColorMUXed {
    private final int deviceNum;
    private final ColorSensorV3 sensor = new ColorSensorV3(MultiplexerConstants.DEFAULT_PORT);

    public ColorMUXed(int deviceNum) {
        this.deviceNum = deviceNum;
    }

    /**
     * @param mode Read/Write (1/0)
     * @return Transfer success.
     * @throws MultiplexerSwitchException
     */
    private boolean setActive(byte mode) throws MultiplexerSwitchException {
        return Multiplexer.getInstance().setDevice(deviceNum, mode);
    }

    /**
     * Attempts to read proximity from color sensor through multiplexer.
     * 
     * @return Sensor's proximity, or -1 upon {@link #MultiplexerSwitchException}
     */
    public int getProximity() {
        try {
            setActive((byte) 1);
            return sensor.getProximity();
        } catch (MultiplexerSwitchException e) {
            e.printStackTrace();
            return -1;
        }
    }
}
