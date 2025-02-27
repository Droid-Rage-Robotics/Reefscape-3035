package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ResetPoseVision;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Carriage.CarriageIntakeValue;
import frc.robot.subsystems.carriage.Carriage.CarriageValue;
import frc.robot.subsystems.drive.SwerveDriveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.utility.shuffleboard.ComplexWidgetBuilder;

public class AutoChooser {
    //Commands
    // Score High Coral
    // Remove algae high and store
    //Intake human
    public static final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    // private final SwerveDrive drive;
    // private final Carriage coralSubsystem;
    // private final Elevator elevator;
    // private final Vision vision;
    public AutoChooser
    (
            SwerveDrive drive, Vision vision)
    // (
    //     SwerveDrive drive, Carriage coralSubsystem, 
    //     Elevator elevator, Vision vision)
     {
        // this.drive = drive;
        // this.coralSubsystem = coralSubsystem;
        // this.elevator = elevator;
        // this.vision = vision;

        NamedCommands.registerCommand("resetPose",
        new ResetPoseVision(drive, vision)
            
        );
         //Put Named Commands HERE
        // NamedCommands.registerCommand("readyShoot",
        //     new ParallelCommandGroup(
        //         algaeSubSystem.setIntakePositionCommand(IntakeValue.READY_SHOOT),
        //         algaeSubSystem.setPositionCommand(ArmValue.SHOOT)
        //     )
        // TODO: Does this work?^^^^^^^
        // );
        // NamedCommands.registerCommand("shoot",
        //     new InstantCommand()
        //     // algaeSubSystem.setIntakePositionCommand(IntakeValue.AUTO_SHOOT)
        // );

        //  NamedCommands.registerCommand("pickGroundAlgae",
        //     new InstantCommand()
        // );

        //  NamedCommands.registerCommand("pickLowAlgae",
        //     new SequentialCommandGroup(
        //         // algaeSubSystem.setIntakePositionCommand(IntakeValue.INTAKE),
        //         // algaeSubSystem.setPositionCommand(ArmValue.LOW)
        //     )
        // );

        //  NamedCommands.registerCommand("pickHighAlgae",
        //     new SequentialCommandGroup(
        //         // algaeSubSystem.setIntakePositionCommand(IntakeValue.INTAKE),
        //         // algaeSubSystem.setPositionCommand(ArmValue.HIGH)
        //     )
        // );

        // NamedCommands.registerCommand("scoreL4",
        //     new SequentialCommandGroup(
        //         // coralSubsystem.setPositionCommand(CarriageValue.L4),
        //         // coralSubsystem.setIntakeCommand(CarriageIntakeValue.OUTTAKE)
        //     )
        // );

        createAutoBuilder(drive);
        ComplexWidgetBuilder.create(autoChooser, "Auto Chooser", "Misc")
            // .
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(1, 3);
        addTuningAuto(drive);

        autoChooser.addOption("NothingAuto", new InstantCommand());
        autoChooser.addOption("VisionTest", Autos.testVision(drive, vision));
        autoChooser.addOption("left", Autos.left(drive, vision));
        autoChooser.addOption("middle", Autos.middle(drive, vision));
        autoChooser.addOption("right", Autos.right(drive, vision));


    }
    
    public  Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static void addTuningAuto(SwerveDrive drive){
        autoChooser.addOption("BackTest", TuningAutos.backTest(drive));
        autoChooser.addOption("ForwardTest", TuningAutos.forwardTest(drive));
        // // autoChooser.addOption("ForwardThenTurnTest", TuningAutos.forwardThenTurnTest(drive));
        autoChooser.addOption("TurnTest", TuningAutos.turnTest(drive));
        autoChooser.setDefaultOption("SplineTest", TuningAutos.splineTest(drive));
        // // autoChooser.addOption("LineToLinearTest", TuningAutos.lineToLinearTest(drive));
        autoChooser.addOption("StrafeRight", TuningAutos.strafeRight(drive));
        autoChooser.addOption("StrafeLeft", TuningAutos.strafeLeft(drive));
        // // autoChooser.addOption("ForwardAndBack", TuningAutos.forwardAndBackTest(drive));
    }

    public static void createAutoBuilder(SwerveDrive drive){
        try {
            // RobotConfig config = new RobotConfig(Units.lbsToKilograms(120), 1, 
            //     null, 
            //     Units.inchesToMeters(29));
            RobotConfig config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder
            AutoBuilder.configure(
                drive::getPose,
                drive::resetOdometry,
                drive::getSpeeds,
                drive::setFeedforwardModuleStates,
                new PPHolonomicDriveController(
                        new PIDConstants(SwerveDriveConstants.SwerveDriveConfig.TRANSLATIONAL_KP.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.TRANSLATIONAL_KI.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.TRANSLATIONAL_KD.getValue()),  // Translation PID constants
                new PIDConstants(SwerveDriveConstants.SwerveDriveConfig.THETA_KP.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.THETA_KI.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.THETA_KD.getValue())),  // Rotation PID constants
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drive // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }

    // public Command autoAlgaePickCommand(){
    //     return new SequentialCommandGroup(
    //         // algaeSubsystem.setIntakePositionCommand(IntakeValue.INTAKE),
    //         // algaeSubsystem.setPositionCommand(ArmValue.AUTO_GROUND),
    //         // new WaitUntilCommand(()-> algaeSubsystem.isAlgaeIn()).withTimeout(2),
    //         // new ParallelCommandGroup(
    //         //     algaeSubsystem.setIntakePositionCommand(IntakeValue.READY_SHOOT),
    //         //     algaeSubsystem.setPositionCommand(ArmValue.SHOOT)
    //         // )
    //     );
    // }
}
