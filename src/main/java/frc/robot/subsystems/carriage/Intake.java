package frc.robot.subsystems.carriage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.DroidRageConstants.Control;
import frc.utility.motor.CANMotorEx;
import frc.utility.motor.TalonEx;
import frc.utility.shuffleboard.ShuffleboardValue;
import frc.utility.motor.CANMotorEx.Direction;
import frc.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.utility.template.IntakeTemplate;

public class Intake extends IntakeTemplate {
    private static class Constants {
        public static final double MAX_SPEED = 800;
        public static final double MIN_SPEED = -800;
    }
    private final ShuffleboardValue<Boolean> isElementInWriter = 
        ShuffleboardValue.create(false, "IsElement", Carriage.class.getSimpleName())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .build();

    private static TalonEx motor = TalonEx.create(31)
        .withDirection(Direction.Reversed)
        .withIdleMode(ZeroPowerMode.Brake)
        .withPositionConversionFactor(1)
        .withSubsystemName(Carriage.class.getSimpleName())
        .withIsEnabled(true)
        .withCurrentLimit(80,80);//60,50

    public Intake(boolean isEnabled) {
        super(
        new CANMotorEx[]{motor}, 
        new PIDController(0.1,0,0), //.15
        // new SimpleMotorFeedforward(0.025, 0.01,0.01),
        new SimpleMotorFeedforward(.0, .6, 0.3),  
        new TrapezoidProfile.Constraints(0, 0),
        Constants.MAX_SPEED, Constants.MIN_SPEED, 
        Control.FEEDFORWARD, Carriage.class.getSimpleName(), "Intake", 0);
        motor.setIsEnabled(isEnabled);
        //Change
    }

    // public Command setPowerCommand(double power){
    //     return new InstantCommand(()->motor.setPower(power));
    // }
    public boolean isElementIn() {
        // return coralLimitSwitch.get();
        isElementInWriter.set(getTargetPosition() - getEncoderPosition() > 40);
        return isElementInWriter.get();
    }
}
