package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public final class Autos {
    public static Command testVision(SwerveDrive drive, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "onePlusFour")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command left(SwerveDrive drive, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "left")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command middle(SwerveDrive drive, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "middle")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command right(SwerveDrive drive, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "right")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    // public static Command partLeft(SwerveDrive drive, Vision vision) {//Top Red/Bottom Blue
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "pLeft")
    //             .setMaxVelocity(6)
    //             .setAcceleration(6)
    //             .build()
    //     );

    // }
    private Autos () {}
}