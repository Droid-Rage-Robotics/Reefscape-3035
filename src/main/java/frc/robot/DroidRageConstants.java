package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystems.carriage.Carriage.CarriageValue;
import frc.utility.shuffleboard.ShuffleboardValue;

public final class DroidRageConstants {
    private final static ShuffleboardValue<String> elementWriter = ShuffleboardValue
        .create("NONE", "Element", "Misc").build();
    public enum Element{
        ALGAE,
        CORAL,
        NONE
    }
    public static Element element = Element.ALGAE;
    // public static InstantCommand flipElement(){
    //     return new InstantCommand(
    //         ()->{
    //             if (element == Element.ALGAE) element = Element.CORAL;
    //             else if (element == Element.CORAL) element = Element.ALGAE;
    //         }
    //     );
    // }
    public static Command setElement(CarriageValue position) {

        return new InstantCommand();
        // return SuppliedCommand.create(
        //     () -> Commands.sequence(
        //         switch(position){
        //             default:
        //                 break;
        //         }

            // Commands.runOnce(() -> logPosition(targetPosition)),
            // switch (position) {
            //         case ALGAE_HIGH, ALGAE_LOW, INTAKE_GROUND:
            //         return Commands.sequence(
            //         new InstantCommand(()->DroidRageConstants.element =
            //         DroidRageConstants.Element.CORAL),
            //         new InstantCommand(()-> elementWriter.set(element.toString()))
            //         );
            //         case INTAKE_HPS, INTAKE_HPS_BLOCK:
            //         return new SequentialCommandGroup(
            //         new InstantCommand(() -> DroidRageConstants.element =
            //         DroidRageConstants.Element.CORAL),
            //         new InstantCommand(() -> elementWriter.set(element.toString())));
            //         case L1,L2,L3,L4,START:
            //         return new InstantCommand();
            //         default:
            //             return new InstantCommand();
                
            // }
        //     )
        // );
        // switch (position){
        //     case ALGAE_HIGH,ALGAE_LOW,INTAKE_GROUND:
        //     return Commands.sequence(
        //         new InstantCommand(()->DroidRageConstants.element = DroidRageConstants.Element.CORAL),
        //         new InstantCommand(()-> elementWriter.set(element.toString()))
        //     );
        //     case INTAKE_HPS, INTAKE_HPS_BLOCK:
        //         return new SequentialCommandGroup(
        //                 new InstantCommand(() -> DroidRageConstants.element = DroidRageConstants.Element.CORAL),
        //                 new InstantCommand(() -> elementWriter.set(element.toString())));
        //     case L1,L2,L3,L4,START:
        //         return new InstantCommand();
        //     default:
        //         return new InstantCommand();
        // }
    }

     
    public static class Gamepad {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double DRIVER_STICK_DEADZONE = 0.05;
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

    public static CANBus driveCanBus = new CANBus("drive");//"drive"
    // public static boolean removeWriter = true; //Can be used to turn off certain writers, hopefulyl preventing loop overruns
    public static ShuffleboardValue<Boolean> removeWriterWriter = 
        ShuffleboardValue.create(true, "RemoveWritersWriter", Robot.class.getSimpleName())
        .withSize(1, 3)
        .build();

    public enum Control{
        PID,
        FEEDFORWARD,
        TRAPEZOID_PROFILE
    }
}
