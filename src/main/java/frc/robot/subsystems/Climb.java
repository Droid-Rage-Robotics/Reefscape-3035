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
        public static final double MAX_POSITION = 300;
        public static final double MIN_POSITION = 50;
        public static final double OFFSET = Math.PI/2;


    }
    
    public static double climb = 68;//48
    public static double hold = 290;
    // public static double climbMore = 60;
//3 4:1s+
//64
//12
    private static TalonEx motor = TalonEx.create(16)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Brake)
        .withPositionConversionFactor( .02)//.02
        //.001 or .01
        //125  and 16:48 //(125/1)*(48/16) //375
        .withSubsystemName("Climb")
        .withIsEnabled(true)
        .withCurrentLimit(50);
        // .0605
    // 0.002666666

    // 0.00037333

    // private static SparkAbsoluteEncoderEx encoder = SparkAbsoluteEncoderEx.create(motor)
    //     .withDirection(EncoderDirection.Forward)
    //     .withOffset(0)
    //     .withSubsystemBase("Climb", Climb.class.getSimpleName());
        
    public Climb(boolean isEnabled) {
        super(
        new CANMotorEx[]{motor}, 
        new PIDController(31,0,0), //kp: 5.2,7
        new ArmFeedforward(0, 0.11, 0.3,0.15), //ks: 0.14 kv:0.1
        new TrapezoidProfile.Constraints(0, 0),
        Constants.MAX_POSITION, Constants.MIN_POSITION, Constants.OFFSET, 
        Control.FEEDFORWARD, "ClimbTAB", "Climb", 0);
        motor.setIsEnabled(isEnabled);
        ComplexWidgetBuilder.create(DisabledCommand.create(runOnce(this::resetEncoder)), "Reset Encoder", this.getName());
        setTargetPosition(90);

    }
}
