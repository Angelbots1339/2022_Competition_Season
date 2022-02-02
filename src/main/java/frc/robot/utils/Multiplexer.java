package frc.robot.utils;

import edu.wpi.first.wpilibj.I2C;
import static frc.robot.Constants.MultiplexerConstants.*;

/**
 * The TCA9548A multiplexer is interesting in that it has an I2C address (0x70
 * by default) - and you basically send it a command to tell it which I2C
 * multiplexed output you want to talk to, then you can address the board you
 * want to address.
 * 
 * https://www.ti.com/lit/ds/symlink/tca9548a.pdf?ts=1643751985463&ref_url=https%253A%252F%252Fwww.google.com%252F
 */
public class Multiplexer extends I2C {
    private static Multiplexer MULTIPLEXER;
    private static final byte[] DEVICE_PORTS = new byte[8];
    private static byte currentDevice;

    private Multiplexer() {
        super(DEFAULT_PORT, DEFAULT_ADDRESS);
        for (int i = 0; i < DEVICE_PORTS.length; i++) {
            DEVICE_PORTS[i] = (byte) (DEFAULT_ADDRESS + i);
        }
        currentDevice = 0;
    }

    /**
     * Returns the byte the selected device number represents.
     * 
     * @param deviceNum Device number (0-7)
     * @return Device's I2C address (0x70-0x77)
     */
    public static byte getDevicePort(int deviceNum) {
        try {
            return DEVICE_PORTS[deviceNum];
        } catch (ArrayIndexOutOfBoundsException e) {
            e.printStackTrace();
            return DEFAULT_ADDRESS;
        }
    }

    public void displayDevices() {
        // Go through each address and write to every i2c port.
        // If any of the registries transfer successfully, the address is valid
        for (int i = 0; i < DEVICE_PORTS.length; i++) {
            System.out.println(new StringBuffer("Multiplexer Port #").append(i).append(" Address #")
                    .append(getDevicePort(i)).toString());

            try {
                setDevice(i, (byte) 1);
                System.out.println("Switch device success");
            } catch (MultiplexerSwitchException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }

            boolean successfulWrite = false;
            byte port;
            for (port = 0; port < 128 && !successfulWrite; port++) {
                if (port == DEFAULT_ADDRESS) {
                    continue;
                }
                if (!getInstance().write(port, 1)) {
                    successfulWrite = true;
                }
            }
            if (successfulWrite) {
                System.out.println(new StringBuffer("Found device on I2C address ")
                        .append(Integer.toHexString(port & 0xFF)).toString());
            }
        }
    }

    /**
     * Configures the multiplexer to pass through the selected device, if it is not
     * already selected
     * 
     * @param device Device on multiplexer to talk to {@link #getDevicePort(int)}
     * @param mode   Read/Write (1/0)
     * @return Transfer success.
     * @throws MultiplexerSwitchException
     */
    public boolean setDevice(byte device, byte mode) throws MultiplexerSwitchException {
        /*
         * v Slave address v
         * {1 ,1, 1 ,0 ,A2,A1,A0,R/W}
         * ^--Fixed--^
         * -------------^Client^
         * ----------------------^RW^
         * RW High = read, RW Low = write
         */
        boolean success = !getInstance().write(mode << device, 1);
        if (!success) {
            throw new MultiplexerSwitchException(device);
        }
        currentDevice = device;
        return success;

    }

    /**
     * Configures the multiplexer to pass through the selected device, if it is not
     * already selected
     * 
     * @param device Device on multiplexer to talk to
     * @param mode   Read/Write (1/0)
     * @return Transfer success.
     * @throws MultiplexerSwitchException
     */
    public boolean setDevice(int device, byte mode) throws MultiplexerSwitchException {
        return setDevice(getDevicePort(device), mode);
    }

    /**
     * 
     * @return Current device address, or 0 if none are selected
     */
    public static byte getCurrentDevice() {
        return currentDevice;
    }

    public static Multiplexer getInstance() {
        if (MULTIPLEXER == null) {
            MULTIPLEXER = new Multiplexer();
        }
        return MULTIPLEXER;
    }

    /**
     * Called by {@link #setDevice(byte,byte)} when data transfer is aborted
     */
    public class MultiplexerSwitchException extends Exception {
        public MultiplexerSwitchException(byte port) {
            super(new StringBuffer("Switch aborted on port ").append(port).toString());
        }
    }
}
