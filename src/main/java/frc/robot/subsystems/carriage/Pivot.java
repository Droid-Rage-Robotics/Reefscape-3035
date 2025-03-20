package frc.robot.subsystems.carriage;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DroidRageConstants.Control;
import frc.utility.encoder.SparkAbsoluteEncoderEx;
import frc.utility.encoder.EncoderEx.EncoderDirection;
import frc.utility.motor.CANMotorEx;
import frc.utility.motor.SparkMaxEx;
import frc.utility.motor.CANMotorEx.Direction;
import frc.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.utility.template.ArmAbsoluteTemplate;

public class Pivot extends ArmAbsoluteTemplate {
    public static class Constants {
        public static final double MAX_POSITION = 243;//200
        public static final double MIN_POSITION = 100;//100
        public static final double OFFSET = Math.PI;
    }
    
    private static SparkMaxEx motor = SparkMaxEx.create(27)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Brake)
        .withPositionConversionFactor(1)
        .withSubsystemName(Carriage.class.getSimpleName())
        .withIsEnabled(true)
        .withCurrentLimit(50);
    
    private static SparkAbsoluteEncoderEx encoder = SparkAbsoluteEncoderEx.create(motor)
        .withDirection(EncoderDirection.Forward)
        // .withPositionConversionFactor(1)
        .withOffset(0)
        .withSubsystemBase("pivot", Carriage.class.getSimpleName());

    public Pivot(boolean isEnabled) {
        super(
        new SparkMaxEx[]{motor}, 
        new PIDController(4, 0, 0), // 2.7 p
        new ArmFeedforward(0.04, 0.21, .25, 0.15),

        // new ArmFeedforward(0.079284, 0.12603, 372.93,
        //                 0.05276), 
        //OLD PROGEJCT - 0.079284, 0.12603, 2.3793, 0.05276
        new TrapezoidProfile.Constraints(0, 0),
        Constants.MAX_POSITION, Constants.MIN_POSITION, Constants.OFFSET, 
        Control.FEEDFORWARD, Carriage.class.getSimpleName(), "Pivot", 0, encoder);
        motor.setIsEnabled(isEnabled);
        
    }
}
