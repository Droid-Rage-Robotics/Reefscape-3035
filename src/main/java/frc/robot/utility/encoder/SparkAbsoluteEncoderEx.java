package frc.robot.utility.encoder;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.EncoderConfig;
import frc.robot.utility.motor.SparkMaxEx;

public class SparkAbsoluteEncoderEx extends EncoderEx {
    protected final SparkAbsoluteEncoder encoder;
    EncoderConfig config = new EncoderConfig();
    
    private SparkAbsoluteEncoderEx(SparkAbsoluteEncoder encoder) {
        this.encoder = encoder;
    }

    public static DirectionBuilder create(SparkMaxEx motor) {
        SparkAbsoluteEncoderEx encoder = new SparkAbsoluteEncoderEx(motor.getAbsoluteEncoder());
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
        return 0;
    }

    @Override
    public void setRange(EncoderRange range) {
        // DOES NOTHING, but it is here for compatibility
    }

    @Override
    public void setOffset(double offset) {
        // DOES NOTHING, but it is here for compatibility
    }
}
