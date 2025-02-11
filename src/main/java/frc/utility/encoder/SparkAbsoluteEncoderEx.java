package frc.utility.encoder;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.EncoderConfig;

import frc.utility.motor.SparkMaxEx;
import lombok.Getter;
import lombok.Setter;

public class SparkAbsoluteEncoderEx extends EncoderEx {
    protected final SparkAbsoluteEncoder encoder;
    private final EncoderConfig config = new EncoderConfig();
    @Getter private double position;
    @Getter private double velocity;
    @Getter private int deviceID;
    @Setter private EncoderDirection direction;
    @Setter private EncoderRange range;
    @Setter private double offset;
    
    

    
    private SparkAbsoluteEncoderEx(SparkAbsoluteEncoder encoder) {
        this.encoder = encoder;
        gettersInit();
        switch (direction) {
            case Reversed -> config.inverted(true);
            case Forward -> config.inverted(false);
        }
    }

    private void gettersInit() {
        position = encoder.getPosition();
        velocity = encoder.getVelocity();
    }

    public static DirectionBuilder create(SparkMaxEx motor) {
        SparkAbsoluteEncoderEx encoder = new SparkAbsoluteEncoderEx(motor.getAbsoluteEncoder());
        return encoder.new DirectionBuilder();
    }
    

    // @Override
    // public double getPosition() { //Raw Position
    //     return encoder.getPosition();
    // }

    // @Override
    // public double getVelocity() {
    //     return encoder.getVelocity();  
    // }

    // @Override
    // public void setDirection(EncoderDirection direction) {
    //     switch (direction) {
    //         case Reversed -> config.inverted(true);
    //         case Forward -> config.inverted(false);
    //     }
    // }

    // @Override
    // public int getDeviceID() {
    //     return 0;
    // }

    // @Override
    // public void setRange(EncoderRange range) {
    //     // DOES NOTHING, but it is here for compatibility
    // }

    // @Override
    // public void setOffset(double offset) {
    //     // DOES NOTHING, but it is here for compatibility
    // }
}
