package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorValue;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Carriage.CarriageIntakeValue;
import frc.robot.subsystems.carriage.Carriage.CarriageValue;

public class AutoCommands{
    public SequentialCommandGroup autoAlgaePickUp(Elevator elevator, Carriage carriage, 
        ElevatorValue elevatorValue, CarriageValue carriageValue) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                carriage.setPositionCommand(carriageValue),
			    elevator.setTargetPositionCommand(elevatorValue),
                carriage.setIntakeCommand(CarriageIntakeValue.INTAKE)
            ),
            new WaitCommand(.37),
            new ParallelCommandGroup(
                elevator.setTargetPositionCommand(elevatorValue.getHeight() + 10)
                // carriage.setIntakeCommand(CarriageIntakeValue.IN)
            )
            // new WaitCommand(1)
        );
    }

    public SequentialCommandGroup teleopHoldCommand(Carriage carriage) {
        return new SequentialCommandGroup(
            // carriage.setIntakeCommand(CarriageIntakeValue.HOLD_CORAL)
            // // new ConditionalCommand(
            // //         carriage.setIntakeCommand(CarriageIntakeValue.HOLD_CORAL),
            // //         carriage.setIntakeCommand(CarriageIntakeValue.HOLD_ALGAE),
            // //         () -> DroidRageConstants.element == DroidRageConstants.Element.CORAL)
        );
    }

    public AutoCommands(){
    }
}
