package frc.utility;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import edu.wpi.first.units.measure.Angle;

public class ServoEx {
    private final boolean isEnabled;
    private final ServoChannel channel;
    private double minPulse,maxPulse,centerPulse;
    private double maxDegree,degRange;
    private boolean centerZeroPoint = false;
    
    
    private ServoEx(ServoChannel channel, boolean isEnabled) {
        this.channel=channel;
        this.isEnabled=isEnabled;
        
        this.channel.setPowered(this.isEnabled);
        this.channel.setEnabled(this.isEnabled);
    }

    /**
     * @param hub the ServoHub object to initialize the servo channel with
     * @param channelId the ID of the channel that the servo is connected to on the servo hub
     * @return a ServoEx object
     */
    public ServoEx create(ServoHub hub,int channelId, boolean isEnabled) {
        new ServoEx(hub.getServoChannel(ChannelId.fromInt(channelId)),isEnabled);
        return this;
    }

    public ServoEx withMinPulseWidth(double value) {
        minPulse=value;
        return this;
    }
    public ServoEx withMaxPulseWidth(double value) {
        maxPulse=value;
        return this;
    }

    /**
     * Used to set the maximum degree of rotation supported by the servo for use in pulse width calculations. 
     * Do not call this method when using a servo with a center zero point, as it will throw an UnsupportedOperationException.
     * @param value maximum degree of rotation
     */
    public ServoEx withDegRange(double value) {
        if(centerZeroPoint){
            throw new UnsupportedOperationException("Cannot use this method with a center zero point!");
        } else {
            degRange=value;
        }
        return this;
    }
    
    /**
     * Used to set the pulse width that represents the zero point on a servo with a center zero point.
     * @param value pulse width, in microseconds
     */
    public ServoEx withCenterPulseWidth(double value) {
        centerPulse=value;
        centerZeroPoint=true;
        return this;
    }

    /**
     * Used when the servo has a center zero point.
     * An example is the REV SRS with -135 deg as min, 0 deg as center, and 135 deg as max. 
     * The value would be set to 135 in this case.
     * @param value the max degrees of rotation in one direction from center
     */
    public ServoEx withMaxDegree(double value) {
        maxDegree=value;
        return this;
    }

    public void setDegrees(double deg) {
        if(centerZeroPoint){
            // Clamp to range -maxDegree .. +maxDegree
            deg = Math.max(-maxDegree, Math.min(maxDegree, deg));

            double pulse;
            if (deg >= 0) {
                pulse = centerPulse + (deg / maxDegree) * (maxPulse - centerPulse);
            } else {
                pulse = centerPulse + (deg / maxDegree) * (centerPulse - minPulse);
            }

            channel.setPulseWidth((int)pulse);
        } else {
            deg = Math.max(0, Math.min(degRange, deg));
            double norm = deg / degRange;
            double pulse = minPulse + norm * (maxPulse - minPulse);
            
            channel.setPulseWidth((int)pulse);
        }  
    }

    public void setRadians(double rad) {
        setDegrees(Math.toDegrees(rad));
    }

    public void setAngle(Angle angle) {
        setDegrees(angle.in(Degrees));
    }
}
