package frc.utility.encoder.wip;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.DroidRageConstants;

public class CANcoderEx {
    private final CANcoder encoder;

    private CANcoderEx(int deviceId, CANBus canBus) {
        this.encoder = new CANcoder(deviceId, canBus);
    }

    /**
     * Creates a new CANcoderEx instance with the specified
     * device id and canbus
     * @param deviceId
     * @param canBus
     * @return a new CANcoderEx instance
     */
    public static CANcoderEx create(int deviceId, CANBus canBus) {
        return new CANcoderEx(deviceId, canBus);
    }
    
    /**
     * Creates a new CANcoderEx instance with the specified
     * device id and the default (rio) canbus.
     * @param deviceId
     * @return a new CANcoderEx instance
     */
    public static CANcoderEx create(int deviceId) {
        return new CANcoderEx(deviceId, DroidRageConstants.rioCanBus);
    }
}