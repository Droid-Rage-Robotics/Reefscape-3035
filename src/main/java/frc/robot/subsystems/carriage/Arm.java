package frc.robot.subsystems.carriage;

import frc.robot.DroidRageConstants.Control;
import frc.utility.encoder.SparkAbsoluteEncoderEx;
import frc.utility.encoder.EncoderEx.EncoderDirection;
import frc.utility.motor.CANMotorEx;
import frc.utility.motor.SparkMaxEx;
import frc.utility.motor.CANMotorEx.Direction;
import frc.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.utility.template.ArmAbsoluteTemplate;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Arm extends ArmAbsoluteTemplate {
    public static class Constants {
        public static final double MAX_POSITION = 215;
        public static final double MIN_POSITION = 45;
        public static final double OFFSET = Math.PI;
    }
    
    private static SparkMaxEx motor = SparkMaxEx.create(17)
        .withDirection(Direction.Reversed)
        .withIdleMode(ZeroPowerMode.Brake)
        .withPositionConversionFactor(1)
        .withSubsystemName("arm")
        .withIsEnabled(true)
        .withCurrentLimit(50);
    
    private static SparkAbsoluteEncoderEx encoder = SparkAbsoluteEncoderEx.create(motor)
        .withDirection(EncoderDirection.Forward)
        // .withPositionConversionFactor(2 * Math.PI)
        .withOffset(0) // Not Used
        .withSubsystemBase("arm", Carriage.class.getSimpleName());
        
    public Arm(boolean isEnabled) {
        super(
        new SparkMaxEx[]{motor}, 
        new PIDController(2.5,0,0), //p= 2.25
        new ArmFeedforward(0.1, 0.25, 0.1, 0.05), 
        new TrapezoidProfile.Constraints(.1, .1),
        Constants.MAX_POSITION, Constants.MIN_POSITION, 
                Constants.OFFSET, 
        Control.FEEDFORWARD, "arm", 0, encoder);
        motor.setIsEnabled(isEnabled);
    }
}
