package frc.robot.commands.SysId;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SysID.SysID;

public class SysIdCommand extends SequentialCommandGroup {
    private final CommandXboxController driver;
    public SysIdCommand(SysID sysId, CommandXboxController driver) {
        this.driver=driver;

        driver.a().onTrue(this);
        driver.b().onTrue(new InstantCommand(()->end(true)));
        
        addCommands(
            sysId.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
            new WaitCommand(1),
            sysId.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(1),
            sysId.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(10),
            new WaitCommand(1),
            sysId.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(10)
        );
    }

    
}