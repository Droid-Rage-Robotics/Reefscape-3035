package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Carriage.CarriageIntakeValue;

public class TeleopCommands{
    public SequentialCommandGroup TeleopOuttakeCommand(Carriage carriage){
        return new SequentialCommandGroup(
            new ConditionalCommand(
                    carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE_L1),
                    carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE),
                    () -> carriage.isL1())
        );
    }

    public SequentialCommandGroup TeleopHoldCommand(Carriage carriage) {
        return new SequentialCommandGroup(
            new ConditionalCommand(
                    carriage.setIntakeCommand(CarriageIntakeValue.HOLD_CORAL),
                    carriage.setIntakeCommand(CarriageIntakeValue.HOLD_ALGAE),
                    () -> DroidRageConstants.element == DroidRageConstants.Element.CORAL));
    }

    public TeleopCommands(){

    }
}
