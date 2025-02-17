package frc.robot.subsystems.carriage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.DroidRageConstants.Control;
import frc.utility.motor.CANMotorEx;
import frc.utility.motor.TalonEx;
import frc.utility.motor.CANMotorEx.Direction;
import frc.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.utility.template.IntakeTemplate;

public class Intake extends IntakeTemplate {
    private static class Constants {
        public static final double MAX_SPEED = 100;
        public static final double MIN_SPEED = -100;
    }
    
    private static TalonEx motor = TalonEx.create(19)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Coast)
        .withPositionConversionFactor(1)
        .withSubsystemName("Carriage/Intake")
        .withIsEnabled(true)
        .withCurrentLimit(50);

    public Intake(boolean isEnabled) {
        super(
        new CANMotorEx[]{motor}, 
        new PIDController(0.03,0,0), 
        new SimpleMotorFeedforward(0.64, 0.000515,0), Constants.MAX_SPEED, Constants.MIN_SPEED, 
        Control.FEEDFORWARD, "Carriage/Intake", 0);
        motor.setIsEnabled(isEnabled);
        //Change
    }
}
