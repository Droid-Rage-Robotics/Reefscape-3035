package frc.utility.encoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utility.shuffleboard.ShuffleboardValue;

public abstract class EncoderEx {
    public enum EncoderRange {
        ZERO_TO_ONE,    //Typically ALWAYS uses 0 to 1
        PLUS_MINUS_HALF
    }
    
    public enum EncoderDirection {
        Forward,
        Reversed,
    }

    protected EncoderDirection direction;
    protected double positionConversionFactor;
    protected double velocityConversionFactor;
    public ShuffleboardValue<Double> degreeWriter;
    public ShuffleboardValue<Double> radianWriter;
    public ShuffleboardValue<Double> rawWriter;
    // public ShuffleboardValue<Boolean> isConnectedWriter;
    public String subsystemName;
    public int deviceID;
    

    public class DirectionBuilder {
        public OffsetBuilder withDirection(EncoderDirection direction) {
            setDirection(direction);
            // return new PositionConversionFactorBuilder();
            return new OffsetBuilder();
        }
    }

    // public class PositionConversionFactorBuilder {
    //     public OffsetBuilder withPositionConversionFactor(double positionConversionFactor) {
    //         setPositionConversionFactor(positionConversionFactor);
    //         return new OffsetBuilder();
    //     }
    // }
    
    public class OffsetBuilder {
        public SubsystemNameBuilder withOffset(double offset) {
            setOffset(offset);
            return new SubsystemNameBuilder();
        }
    }

    public class SubsystemNameBuilder {
        public <T extends EncoderEx> T withSubsystemBase(String encoderName, SubsystemBase subsystemBase) {
            return withSubsystemBase(encoderName, subsystemBase.getClass().getSimpleName());
        }
        
        @SuppressWarnings("unchecked")
        public <T extends EncoderEx> T withSubsystemBase(String encoderName, String subsystemBase) {
            subsystemName = subsystemBase;
            rawWriter = ShuffleboardValue
                    .create(0.0, subsystemName + "/" + encoderName + "/Pos/Raw", subsystemName)
                    .withSize(1, 2)
                    .build();
            degreeWriter = ShuffleboardValue
                    .create(0.0, subsystemName + "/" + encoderName + "/Pos/Degree", subsystemName)
                    .withSize(1, 2)
                    .build();
            radianWriter = ShuffleboardValue
                    .create(0.0, subsystemName + "/" + encoderName + "/Pos/Radian", subsystemName)
                    .withSize(1, 2)
                    .build();
            return (T) EncoderEx.this;
        }
    }

    // public class RangeBuilder {
    //     @SuppressWarnings("unchecked")
    //     public <T extends EncoderEx> T withRange(EncoderRange range) {
    //         setRange(range);
    //         return (T) EncoderEx.this;
    //     }
    // }
    public CANcoderEx withRange(EncoderRange range){
        setRange(range);
        return (CANcoderEx) this;
    }
    
    public void periodic() {
        rawWriter.set(getPosition());
        degreeWriter.set(getDegree());
        radianWriter.set(getRadian());
    }
    public abstract double getVelocity();
    public abstract double getPosition();
    
    // Probably not work
    // public double getDegree(){
    //     return getPosition() * 360;
    // }    
    // public double getRadian() {
    //     return getPosition() * (2*Math.PI);
    // }

    /**
     * @param offset the offset in degrees to include in calculations
     * @return encoder position in degrees
     */
    public double getDegree(double offset) {
        return MathUtil.inputModulus((getPosition() * 360) + offset, 0, 360);
    }

    /**
     * 
     * @return encoder position in degrees
     */
    public double getDegree() {
        return MathUtil.inputModulus(getPosition() * 360, 0, 360);
    } 

    /**
     * @param offset the radian offset to include in calculations
     * @return encoder position in radians
     */
    public double getRadian(double offset) {
        return MathUtil.inputModulus((getPosition() * (2 * Math.PI)) + offset, 0, (2 * Math.PI));
    }

    /**
     *
     * @return encoder position in radians
     */
    public double getRadian() {
        return MathUtil.inputModulus(getPosition() * (2 * Math.PI), 0, (2 * Math.PI));
    }

    protected void setPositionConversionFactor(double positionConversionFactor) {
        this.positionConversionFactor = positionConversionFactor;
    }

    protected void setVelocityConversionFactor(double velocityConversionFactor) {
        this.velocityConversionFactor = velocityConversionFactor;
    }
    public abstract void setDirection(EncoderDirection direction);
    public abstract int getDeviceID();
    public abstract void setRange(EncoderRange range);
    public abstract void setOffset(double offset);
}
