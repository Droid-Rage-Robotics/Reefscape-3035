package frc.utility.encoder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.subsystems.drive.SwerveDrive;
import frc.utility.shuffleboard.ShuffleboardValue;

public class CANcoderEx extends EncoderEx {
    private final CANcoder encoder;
    private final CANcoderConfiguration config;
    private final CANcoderConfigurator configurator;
    private double positionConversionFactor, velocityConversionFactor;
    // private final ShuffleboardValue<String> speedStateWriter;
    public CANcoderEx(CANcoder encoder) {
        this.encoder = encoder;
        this.config = new CANcoderConfiguration();
        this.configurator = encoder.getConfigurator();

        // speedStateWriter = ShuffleboardValue.create("test", "Current/State/Speed", "Swerve").build();

    }

    public static DirectionBuilder create(int deviceID, CANBus canbus) {
        
        CANcoderEx encoder = new CANcoderEx(new CANcoder(deviceID, canbus));
        
        encoder.deviceID = deviceID;
        return encoder.new DirectionBuilder();
    }
    
    public static DirectionBuilder create(int deviceID) {
        CANcoderEx encoder = new CANcoderEx(new CANcoder(deviceID));
        encoder.deviceID = deviceID;
        return encoder.new DirectionBuilder();
    }

    @Override
    public void setDirection(EncoderDirection direction) {
        switch (direction) {
            case Reversed:
                config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            case Forward:
                config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        }
        configurator.apply(config);
    }

    public void setRange(EncoderRange range) {
        switch (range) {
            case PLUS_MINUS_HALF:
                config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            case ZERO_TO_ONE:
                config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        }
        configurator.apply(config);
    }

    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }
    
    @Override
    public double getVelocity() {
        return encoder.getVelocity().getValueAsDouble() * velocityConversionFactor;
    }

    @Override
    public double getPosition() {
        return encoder.getPosition().getValueAsDouble() * positionConversionFactor;
    }

    @Override
    public int getDeviceID() {
        return encoder.getDeviceID();
    }

    @Override
    public void setOffset(double offset) {
        config.MagnetSensor.MagnetOffset = offset;
        configurator.apply(config);
    }
}


