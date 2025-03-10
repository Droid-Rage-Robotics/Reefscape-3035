package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorValue;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Carriage.CarriageIntakeValue;
import frc.robot.subsystems.carriage.Carriage.CarriageValue;
import frc.utility.template.ElevatorTemplate;

public class TeleopCommands{
    public SequentialCommandGroup teleopOuttakeCommand(Carriage carriage){
        return new SequentialCommandGroup(
            new ConditionalCommand(
                    carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE_L1),
                    carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE),
                    () -> carriage.isL1())
        );
    }

    public SequentialCommandGroup teleopHoldCommand(Carriage carriage) {
        return new SequentialCommandGroup(
            carriage.setIntakeCommand(CarriageIntakeValue.HOLD_CORAL)
            // new ConditionalCommand(
            //         carriage.setIntakeCommand(CarriageIntakeValue.HOLD_CORAL),
            //         carriage.setIntakeCommand(CarriageIntakeValue.HOLD_ALGAE),
            //         () -> DroidRageConstants.element == DroidRageConstants.Element.CORAL)
        );
    }
    
    public SequentialCommandGroup runIntakeFor(Carriage carriage, CarriageIntakeValue value, double waitSec) {
        return new SequentialCommandGroup(
            carriage.setIntakeCommand(value),
            new WaitCommand(waitSec),
            carriage.setIntakeCommand(CarriageIntakeValue.STOP)
        );
    }

    // public SequentialCommandGroup outtakeAndGrabHigh(Elevator elevator, Carriage carriage, ElevatorValue value ){
    //     return new SequentialCommandGroup(
    //         carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE),
    //         new WaitCommand(1.3),
    //         new ParallelCommandGroup(
    //             // carriage.setIntakeCommand(CarriageIntakeValue.STOP),
    //             carriage.setIntakeCommand(CarriageIntakeValue.INTAKE),
    //             carriage.setPositionCommand(CarriageValue.ALGAE_HIGH),
    //             elevator.setPositionCommand(value)
    //         ),
    //         // new WaitCommand(.5),
    //         new WaitCommand(1.5),
    //         new ParallelCommandGroup(
    //             carriage.setIntakeCommand(CarriageIntakeValue.STOP),
    //             carriage.setPositionCommand(CarriageValue.HOLD),
    //             elevator.setPositionCommand(ElevatorValue.GROUND)
    //         )
    //     );
    // }

    public TeleopCommands(){
    }
}
