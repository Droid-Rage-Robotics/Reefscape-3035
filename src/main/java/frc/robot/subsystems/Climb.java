package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DroidRageConstants.Control;
import frc.robot.commands.DisabledCommand;
import frc.utility.encoder.SparkAbsoluteEncoderEx;
import frc.utility.encoder.EncoderEx.EncoderDirection;
import frc.utility.motor.CANMotorEx;
import frc.utility.motor.SparkMaxEx;
import frc.utility.motor.TalonEx;
import frc.utility.shuffleboard.ComplexWidgetBuilder;
import frc.utility.motor.CANMotorEx.Direction;
import frc.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.utility.template.ArmAbsoluteTemplate;
import frc.utility.template.ArmTemplate;

public class Climb extends ArmTemplate {
    public static class Constants {
        public static final double MAX_POSITION = 15;
        public static final double MIN_POSITION = 0;
        public static final double OFFSET = 0;


    }
    
    public static double climb = 15;
    public static double hold = 0;
//3 4:1s
//64
//12
    private static TalonEx motor = TalonEx.create(16)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Coast)
        .withPositionConversionFactor(1)//125  and 16:48 //(125/1)*(16/48)
        .withSubsystemName("Climb")
        .withIsEnabled(true)
        .withCurrentLimit(50);
    
    // private static SparkAbsoluteEncoderEx encoder = SparkAbsoluteEncoderEx.create(motor)
    //     .withDirection(EncoderDirection.Forward)
    //     .withOffset(0)
    //     .withSubsystemBase("Climb", Climb.class.getSimpleName());
        
    public Climb(boolean isEnabled) {
        super(
        new CANMotorEx[]{motor}, 
        new PIDController(0,0,0), //3.5p
        new ArmFeedforward(0, 0, 0,0), //0.46, .655, .0859,.003587
        new TrapezoidProfile.Constraints(0, 0),
        Constants.MAX_POSITION, Constants.MIN_POSITION, Constants.OFFSET, 
        Control.PID, "Climb", 0);
        motor.setIsEnabled(isEnabled);

         ComplexWidgetBuilder.create(DisabledCommand.create(runOnce(this::resetEncoder)), "Reset Encoder", this.getName());

    }
}
