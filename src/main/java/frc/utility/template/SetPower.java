package frc.utility.template;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.utility.motor.CANMotorEx;
import frc.utility.shuffleboard.ShuffleboardValue;

//Works
public class SetPower {
    private final CANMotorEx[] motors;
    private final ShuffleboardValue<Double> powerWriter;

    public SetPower(
        CANMotorEx[] motors,
        String name
    ){
        this.motors=motors;

        powerWriter = ShuffleboardValue
            .create(0.0, name+"/Power", name)
            .build();
    }

    public Command setTargetPowerCommand(double power){
        return new InstantCommand(()->setTargetPower(power));
    }

    /*
     * Use this for initialization
     */
    public void setTargetPower(double power) {
        powerWriter.set(power);
        for (CANMotorEx motor: motors) {
            motor.setPower(power);
        }
    }
}
