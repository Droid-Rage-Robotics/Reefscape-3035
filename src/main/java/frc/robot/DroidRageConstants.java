package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystems.carriage.Carriage.CarriageValue;
import frc.robot.subsystems.vision.Vision;
import frc.utility.shuffleboard.ShuffleboardValue;

public final class DroidRageConstants {
    public enum Alignment {
        RIGHT,
        LEFT,
        MIDDLE
    }
    private final static ShuffleboardValue<String> alignmentWriter = ShuffleboardValue
        .create(Alignment.MIDDLE.toString(), "Alignment", Vision.class.getSimpleName()).build();
    public static Alignment alignmentMode = Alignment.MIDDLE;
    
    public static void setAlignment(Alignment alignment){
        alignmentMode = alignment;
        alignmentWriter.set(alignmentMode.toString());
    }

    
    private final static ShuffleboardValue<String> elementWriter = ShuffleboardValue
        .create(Element.NONE.toString(), "Element", "Misc").build();
    //All possible elements
        public enum Element{
        ALGAE,
        CORAL,
        NONE
    }
    public static Element element = Element.ALGAE;
    public static Command setElement(CarriageValue position){
        return SuppliedCommand.create(
            () -> Commands.sequence(
            switch(position){
                case ALGAE_HIGH, ALGAE_LOW, BARGE, BARGE_HOLD, INTAKE_GROUND, PROCESSOR:
                    yield new SequentialCommandGroup(
                        new InstantCommand(()-> element = Element.ALGAE),
                        new InstantCommand(()->elementWriter.set(element.toString()))
                    );
                case INTAKE_HPS, INTAKE_HPS_BLOCK, L1, L2, L3,L4:
                    yield new SequentialCommandGroup(
                        new InstantCommand(()-> element = Element.CORAL),
                        new InstantCommand(()->elementWriter.set(element.toString()))
                    );
                default:
                    yield new SequentialCommandGroup(
                        new InstantCommand(()-> element = Element.NONE),
                        new InstantCommand(()->elementWriter.set(element.toString()))
                    );
            }
        ));
        // switch(position){
        //     case ALGAE_HIGH, ALGAE_LOW, INTAKE_GROUND:
        //         DroidRageConstants.element = DroidRageConstants.Element.ALGAE;
        //         elementWriter.set(element.toString());
        //         break;
        //     case INTAKE_HPS, INTAKE_HPS_BLOCK:
        //         DroidRageConstants.element = DroidRageConstants.Element.CORAL;
        //         elementWriter.set(element.toString());
        //         break;
        //     default:
        //         DroidRageConstants.element = DroidRageConstants.Element.NONE;
        //         elementWriter.set(element.toString());
        //         break;
        // }
    }
    
    public static class Gamepad {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double DRIVER_STICK_DEADZONE = 0.025;
        public static final double OPERATOR_STICK_DEADZONE = 0.2;
    }

    public static double LOOP_TYPE_SECONDS = 0.02;

    public static double squareInput(double value) {
        return value * Math.abs(value);
    }

    public static double applyDeadBand(double value) {
        if (Math.abs(value) < DroidRageConstants.Gamepad.OPERATOR_STICK_DEADZONE) value = 0;
        return value;
    }

    public static boolean isWithinDeadzone(double stick) {
        return Math.abs(stick) < DroidRageConstants.Gamepad.OPERATOR_STICK_DEADZONE;
    }

    public static CANBus driveCanBus = new CANBus("drive");
    public static String leftLimelight = "limelight-left";
    public static String rightLimelight = "limelight-right";

    public enum Control{
        PID,
        FEEDFORWARD,
        TRAPEZOID_PROFILE
    }
}
