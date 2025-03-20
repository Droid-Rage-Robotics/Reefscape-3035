package frc.robot;

import com.ctre.phoenix6.CANBus;

import frc.utility.shuffleboard.ShuffleboardValue;

public final class DroidRageConstants {
    public enum Alignment {
        RIGHT,
        LEFT,
    }
    public static Alignment alignmentMode = Alignment.LEFT;
    public static void flipAlignment() {
        if(alignmentMode == Alignment.RIGHT) {
            alignmentMode = Alignment.LEFT;
        } else {
            alignmentMode = Alignment.RIGHT;
        }
    }
    // private final static ShuffleboardValue<String> elementWriter = ShuffleboardValue
    //     .create(Element.NONE.toString(), "Element", "Misc").build();
    // //All possible elements
    //     public enum Element{
    //     ALGAE,
    //     CORAL,
    //     NONE
    // }
    // public static Element element = Element.ALGAE;
    // public static void setElement(CarriageValue position){
    //     switch(position){
    //         case ALGAE_HIGH, ALGAE_LOW, INTAKE_GROUND:
    //             DroidRageConstants.element = DroidRageConstants.Element.ALGAE;
    //             elementWriter.set(element.toString());
    //             break;
    //         case INTAKE_HPS, INTAKE_HPS_BLOCK:
    //             DroidRageConstants.element = DroidRageConstants.Element.CORAL;
    //             elementWriter.set(element.toString());
    //             break;
    //         default:
    //             DroidRageConstants.element = DroidRageConstants.Element.NONE;
    //             elementWriter.set(element.toString());
    //             break;
    //     }
    // }
    
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
    // public static boolean removeWriter = true; //Can be used to turn off certain writers, hopefulyl preventing loop overruns
    public static ShuffleboardValue<Boolean> removeWriterWriter = 
        ShuffleboardValue.create(false, "RemoveWritersWriter", Robot.class.getSimpleName())
        .withSize(1, 3)
        .build();

    public enum Control{
        PID,
        FEEDFORWARD,
        TRAPEZOID_PROFILE
    }
}
