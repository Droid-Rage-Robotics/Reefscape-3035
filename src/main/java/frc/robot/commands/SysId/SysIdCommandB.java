package frc.robot.commands.SysId;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdCommandB extends SequentialCommandGroup {
    public SysIdCommandB(SysIdRoutine routine) {
        addCommands(
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            new WaitCommand(1),
            routine.quasistatic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(1),
            routine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(10),
            new WaitCommand(1),
            routine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(10)
        );
    }
}