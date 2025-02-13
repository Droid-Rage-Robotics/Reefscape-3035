package frc.utility.encoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import lombok.Getter;
import lombok.Setter;

public class AbsoluteDutyEncoderRIO extends EncoderEx {
    private final DutyCycleEncoder encoder;
    @Getter(onMethod = @__(@Override)) private final int deviceID; // Should work
    @Setter(onMethod = @__(@Override)) private double offset = 0; // Should work
    @Getter(onMethod = @__(@Override)) private double velocity; // No use; here for compatibility
    @Setter(onMethod = @__(@Override)) private EncoderRange range; // No use; here for compatibility
    
    public String name;
    
    private AbsoluteDutyEncoderRIO(DutyCycleEncoder encoder, int deviceID){
        this.encoder=encoder;
        this.deviceID = deviceID;
        encoder.setAssumedFrequency(975.6);
    }
    
    public static DirectionBuilder create(int deviceID) {
        AbsoluteDutyEncoderRIO encoder = new AbsoluteDutyEncoderRIO(
            new DutyCycleEncoder(deviceID), deviceID);
        return encoder.new DirectionBuilder();
    }

    @Override
    public void setDirection(EncoderDirection direction) {
        switch (direction) {
            case Forward -> encoder.setInverted(false);
            case Reversed -> encoder.setInverted(true);
        }
        this.direction = direction;
    }

    @Override
    public double getPosition() {
        double givenPos = encoder.get();

        // Handle direction inversion
        if (direction == EncoderDirection.Reversed) {
            givenPos = 1 - givenPos;
        }

        // Correct for rollover (values between 0 and 1)
        if (givenPos < 0) {
            givenPos = 1 + givenPos;  // Wrap around positive
        } else if (givenPos >= 1) {
            givenPos = givenPos - 1;  // Wrap around to stay between 0 and 1
        }

        return givenPos - offset;
    }
}