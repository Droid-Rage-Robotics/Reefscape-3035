package frc.utility.encoder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.subsystems.drive.SwerveDrive;
import frc.utility.shuffleboard.ShuffleboardValue;

public class CANcoderEx extends EncoderEx {
    private final CANcoder encoder;
    private CANcoderConfiguration config = new CANcoderConfiguration();
    private double positionConversionFactor, velocityConversionFactor;
    // private final ShuffleboardValue<String> speedStateWriter;
    public CANcoderEx(CANcoder encoder) {
        this.encoder = encoder;
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
        encoder.getConfigurator().apply(config);

        //config.MotorOutput.Inverted = switch (direction) {
        //     case Forward -> InvertedValue.Clockwise_Positive;
        //     case Reversed -> InvertedValue.CounterClockwise_Positive;
        // };
    }

    public void setRange(EncoderRange range) {
        switch (range) {
            case PLUS_MINUS_HALF:
                config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            case ZERO_TO_ONE:
                config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        }
        encoder.getConfigurator().apply(config);
    }

    // public void setMagnetSensorOffset(double offset) {
    //     config.MagnetSensor.MagnetOffset = offset;
    // }

    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }
    
    public double getVelocity() {
        return encoder.getVelocity().getValueAsDouble() * velocityConversionFactor;
    }

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
        encoder.getConfigurator().apply(config);
    }
}


