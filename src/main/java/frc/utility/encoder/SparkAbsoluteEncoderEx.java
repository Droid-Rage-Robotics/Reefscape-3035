package frc.utility.encoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.EncoderConfig;

import frc.utility.motor.SparkMaxEx;
import frc.utility.motor.SparkMaxEx.EncoderType;
import lombok.Setter;

public class SparkAbsoluteEncoderEx extends EncoderEx {
    protected final AbsoluteEncoder encoder;
    protected final SparkMaxEx motor;
    private final EncoderConfig config = new EncoderConfig();
    private double offset;
    @Setter(onMethod = @__(@Override)) private EncoderRange range; // No use; here for compatibility
    
    private SparkAbsoluteEncoderEx(AbsoluteEncoder encoder, SparkMaxEx motor) {
        this.encoder = encoder;
        this.motor = motor;
    }

    public static DirectionBuilder create(SparkMaxEx motor) {
        SparkAbsoluteEncoderEx encoder = new SparkAbsoluteEncoderEx(motor.getEncoder(EncoderType.Absolute), motor);
        return encoder.new DirectionBuilder();
    }
    
    @Override
    public void setOffset(double offset) {
        this.offset = offset;
    }

    @Override
    public double getPosition() { //Raw Position
        // config.positionConversionFactor(positionConversionFactor);
        // double rawPos = encoder.getPosition();
        // double convertedPos = (rawPos - offset) % positionConversionFactor;

        // if(convertedPos < 0) {
        //     convertedPos += positionConversionFactor;
        // }

        // return convertedPos;
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
