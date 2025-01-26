package frc.robot.subsystems.carriage;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.DroidRageConstants.Control;
import frc.robot.utility.encoder.EncoderEx.EncoderDirection;
import frc.robot.utility.encoder.SparkAbsoluteEncoderEx;
import frc.robot.utility.motor.CANMotorEx;
import frc.robot.utility.motor.CANMotorEx.Direction;
import frc.robot.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.robot.utility.motor.SparkMaxEx;
import frc.robot.utility.template.ArmAbsoluteTemplate;

public class Pivot extends ArmAbsoluteTemplate {
    public static class Constants {
        public static final double MAX_POSITION = 0;
        public static final double MIN_POSITION = 0;
        public static final double OFFSET = 0;
    }
    
    private static SparkMaxEx motor = SparkMaxEx.create(53)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Coast)
        .withPositionConversionFactor(1)
        .withSubsystemName("carriage")
        .withIsEnabled(true)
        .withCurrentLimit(50);
    
    private static SparkAbsoluteEncoderEx encoder = SparkAbsoluteEncoderEx.create(motor)
        .withDirection(EncoderDirection.Forward)
        .withOffset(0)
        .withSubsystemBase("pivot", Carriage.class.getSimpleName());

    public Pivot(boolean isEnabled) {
        super(
        new CANMotorEx[]{motor}, 
        new PIDController(0,0,0), 
        new ArmFeedforward(0, 0, 0, 0, 0), 
        Constants.MAX_POSITION, Constants.MIN_POSITION, Constants.OFFSET, 
        Control.PID, "carriage", 0, encoder);
        motor.setIsEnabled(isEnabled);
        
    }
}
