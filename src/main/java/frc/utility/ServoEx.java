package frc.utility;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;

public class ServoEx {
    private final ServoChannel channel;
    private double min,max;    
    
    private ServoEx(ServoChannel channel) {
        this.channel=channel;
    }

    // public void setDegrees(double deg) {
    //     deg = Math.max(0, Math.min(degRange, deg));
    //     double norm = deg / degRange;
    //     double pulseUs = min + norm * (max - min);
    //     channel.setPulseWidth((int)pulseUs);
    // }

    public ServoEx create(ServoHub hub,int channelId) {
        new ServoEx(hub.getServoChannel(ChannelId.fromInt(channelId)));
        return this;
    }

    public ServoEx withMinPulseWidth(double value) {
        min=value;
        return this;
    }
    public ServoEx withMaxPulseWidth(double value) {
        max=value;
        return this;
    }
}
