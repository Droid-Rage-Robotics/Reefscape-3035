package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Carriage.CarriageIntakeValue;

public class TeleopIntakeCommand extends SequentialCommandGroup{
    public TeleopIntakeCommand(Carriage carriage){
        addCommands(
            new ConditionalCommand(
                    carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE_L1),
                    carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE),
                    () -> carriage.isL1())
        );
    }
}
