package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.DroidRageConstants.Control;
import frc.utility.motor.CANMotorEx;
import frc.utility.motor.TalonEx;
import frc.utility.motor.CANMotorEx.Direction;
import frc.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.utility.template.ElevatorTemplate;

public class Elevator extends ElevatorTemplate {
    public static class Constants {
        public static final double MIN_POSITION = 0;
        public static final double MAX_POSITION = 10;   
    }

    public enum ElevatorValue {
        START(0),
        GROUND(0),
        INTAKE_HPS(0),
        CLIMB(1),
        
        L1(2),
        L2(3),
        L3(4),
        L4(0),
        LOW(0),
        HIGH(0)
        ;

        private final double height;

        private ElevatorValue(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }

    private static TalonEx motorRight = TalonEx.create(70)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Coast)
        .withPositionConversionFactor(1)
        .withSubsystemName("Elevator")
        .withIsEnabled(true)
        .withCurrentLimit(50);

    private static TalonEx motorLeft = TalonEx.create(70)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Coast)
        .withPositionConversionFactor(1)
        .withSubsystemName("Elevator")
        .withIsEnabled(true)
        .withCurrentLimit(50);
    
    private static TalonEx[] motors = {motorRight, motorLeft};
    
    public Elevator(boolean isEnabled) {
        super(
        new CANMotorEx[]{motorRight, motorLeft}, 
        new PIDController(0, 0, 0), 
        new ElevatorFeedforward(0, 0, 0, 0), 
        Constants.MAX_POSITION, 
        Constants.MIN_POSITION, 
        Control.PID, "Elevator", 0);
        for (TalonEx motor: motors) {
            motor.setIsEnabled(isEnabled);
        }
    }

    public Command setPositionCommand(ElevatorValue target) {
        return new InstantCommand(()->motorRight.setPower(1));
    }
}
