package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.carriage.Carriage;
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
    public static Command leftOnePlusTwo(SwerveDrive drive, Elevator elevator, Carriage carriage,Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "left1+2")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command leftOnePlusOne(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "left1+1")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command leftOne(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "left1")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command middleProcessor(SwerveDrive drive, Elevator elevator, Carriage carriage,Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "middleProcessor")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command middleBarge(SwerveDrive drive, Elevator elevator, Carriage carriage,Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "middleBarge")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command rightOnePlusTwo(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "right1+2")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command rightOnePlusOne(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "right1+1")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command rightOne(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "right1")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command middleL1(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "middleL1")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command middle(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision, String score) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "middle" + score)
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command middleL4Algae(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "middleL4Algae")
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