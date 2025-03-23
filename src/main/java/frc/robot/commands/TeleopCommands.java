package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorValue;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Carriage.CarriageIntakeValue;
import frc.robot.subsystems.carriage.Carriage.CarriageValue;

public class TeleopCommands{
    public SequentialCommandGroup teleopOuttakeCommand(Carriage carriage){
        return new SequentialCommandGroup(
            new ConditionalCommand(
                carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE_L1),
                    new ConditionalCommand(
                            carriage.setIntakeCommand(CarriageIntakeValue.SHOOT),
                            carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE),
                            () -> carriage.isCarriageValue(CarriageValue.BARGE)),
                () -> carriage.isCarriageValue(CarriageValue.L1))
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

    public SequentialCommandGroup barge(Elevator elevator, Carriage carriage){
        return new SequentialCommandGroup(
            carriage.setPositionCommand(CarriageValue.BARGE_HOLD),
            new WaitUntilCommand(()->
                Math.abs(carriage.getArm().getTargetPosition()-carriage.getArm().getEncoderPosition())<3),
            elevator.setTargetPositionCommand(ElevatorValue.BARGE),
            new WaitUntilCommand(()->elevator.getEncoderPosition()>=47),
            // new WaitCommand(1),
            carriage.setPositionCommand(CarriageValue.BARGE)
        );
    }
    public SequentialCommandGroup intakeHPS(Elevator elevator, Carriage carriage, CarriageValue value){
        return new SequentialCommandGroup(
            elevator.setTargetPositionCommand(ElevatorValue.INTAKE_HPS),
            new WaitUntilCommand(()->elevator.getEncoderPosition()<=2),
            carriage.setPositionCommand(value)
            // carriage.setIntakeCommand(CarriageIntakeValue.INTAKE)
        );
    }

    public SequentialCommandGroup resetCarriageFromBarge(Elevator elevator, Carriage carriage){
        // return new SequentialCommandGroup(
        //     carriage.setPositionCommand(CarriageValue.INTAKE_HPS),
        //     elevator.setTargetPositionCommand(ElevatorValue.GROUND)
        // );
        return new SequentialCommandGroup(
            carriage.setPositionCommand(CarriageValue.L4),
            new WaitUntilCommand(()->carriage.getArm().atSetpoint()),
            new WaitUntilCommand(()->carriage.getPivot().atSetpoint()),
            elevator.setTargetPositionCommand(ElevatorValue.GROUND)    
            // new ConditionalCommand(
            //     new SequentialCommandGroup(
            //         carriage.setPositionCommand(CarriageValue.BARGE_HOLD),
            //         new WaitUntilCommand(()->carriage.getArm().atSetpoint()),
            //         new WaitUntilCommand(()->carriage.getPivot().atSetpoint()),
            //         elevator.setTargetPositionCommand(ElevatorValue.GROUND)                    
            //     ),
            //     new SequentialCommandGroup(
            //         intakeHPS(elevator, carriage, CarriageValue.INTAKE_HPS)
            //     ),
            //    ()-> carriage.isCarriageValue(CarriageValue.BARGE)
            // )
        );
    }

    public SequentialCommandGroup goL4(Elevator elevator, Carriage carriage){
        return new SequentialCommandGroup(
            carriage.getArm().setTargetPositionCommand(CarriageValue.L4.getArmAngle()),
            new WaitUntilCommand(()->carriage.getArm().atSetpoint()),
            new ParallelCommandGroup(
                elevator.setTargetPositionCommand(ElevatorValue.L4),
                carriage.getPivot().setTargetPositionCommand(CarriageValue.L4.getPivotAngle())
            )
        );
    }

    public TeleopCommands(){
    }
}
