package frc.utility.encoder;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.EncoderConfig;

import frc.utility.motor.SparkMaxEx;
import lombok.Setter;

public class SparkAbsoluteEncoderEx extends EncoderEx {
    protected final SparkAbsoluteEncoder encoder;
    protected final SparkMaxEx motor;
    private final EncoderConfig config = new EncoderConfig();
    @Setter(onMethod = @__(@Override)) private EncoderRange range; // No use; here for compatibility
    @Setter(onMethod = @__(@Override)) private double offset; // No use; here for compatibility
    
    private SparkAbsoluteEncoderEx(SparkAbsoluteEncoder encoder, SparkMaxEx motor) {
        this.encoder = encoder;
        this.motor = motor;
    }

    public static DirectionBuilder create(SparkMaxEx motor) {
        SparkAbsoluteEncoderEx encoder = new SparkAbsoluteEncoderEx(motor.getAbsoluteEncoder(), motor);
        return encoder.new DirectionBuilder();
    }
    

    @Override
    public double getPosition() { //Raw Position
        return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();  
    }

    @Override
    public void setDirection(EncoderDirection direction) {
        switch (direction) {
            case Reversed -> config.inverted(true);
            case Forward -> config.inverted(false);
        }
    }

    @Override
    public int getDeviceID() {
        return motor.getDeviceID();
    }
}
