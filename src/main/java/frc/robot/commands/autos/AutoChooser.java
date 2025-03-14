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
    // private final Carriage carriage;
    // private final Elevator elevator;
    // private final Vision vision;
    public AutoChooser(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision)
     {
        // this.drive = drive;
        // this.carriage = carriage;
        // this.elevator = elevator;
        // this.vision = vision;

        NamedCommands.registerCommand("resetPose",
            new ResetPoseVision(drive, vision) 
        );
        NamedCommands.registerCommand("shoot",
            new ParallelCommandGroup(
                carriage.setIntakeCommand(CarriageIntakeValue.SHOOT),
                elevator.setTargetPositionCommand(Elevator.ElevatorValue.BARGE)
            )
        );
        NamedCommands.registerCommand("placeProcessor",
            new ParallelCommandGroup(
                elevator.setTargetPositionCommand(Elevator.ElevatorValue.GROUND),
                carriage.setPositionCommand(CarriageValue.PROCESSOR),
                carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE)
            )
        );
                

        NamedCommands.registerCommand("pickLAlgae",
            new AutoCommands().autoAlgaePickUp(elevator, carriage, 
                Elevator.ElevatorValue.ALGAE_LOW, CarriageValue.ALGAE_LOW)
        );

        NamedCommands.registerCommand("pickHAlgae",
            new AutoCommands().autoAlgaePickUp(elevator, carriage, 
                Elevator.ElevatorValue.ALGAE_HIGH, CarriageValue.ALGAE_HIGH)
        );

        NamedCommands.registerCommand("placeL4",
            new SequentialCommandGroup(
                carriage.setPositionCommand(CarriageValue.L4),
                new WaitCommand(.5),
                elevator.setTargetPositionCommand(Elevator.ElevatorValue.L4),
                new WaitCommand(1),
                carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE),
                new WaitCommand(3)
            )
        );
        NamedCommands.registerCommand("resetCarriage",
            new ParallelCommandGroup(
                elevator.setTargetPositionCommand(Elevator.ElevatorValue.GROUND),
                carriage.setPositionCommand(CarriageValue.HOLD),
                carriage.setIntakeCommand(CarriageIntakeValue.HOLD_CORAL)
            )
        );
        NamedCommands.registerCommand("placeBarge",
            new SequentialCommandGroup(
                new WaitCommand(1.),
                new ParallelCommandGroup(
                    elevator.setTargetPositionCommand(Elevator.ElevatorValue.BARGE),
                    carriage.setPositionCommand(CarriageValue.BARGE),
                    carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE)
                )
            )
        );
        NamedCommands.registerCommand("intake",
            new SequentialCommandGroup(
                new WaitCommand(4),
                new ParallelCommandGroup(
                    elevator.setTargetPositionCommand(Elevator.ElevatorValue.L4),
                    carriage.setPositionCommand(CarriageValue.L4)
                )
            )   
            
        );
        NamedCommands.registerCommand("stop",
            new WaitCommand(3)
        );
        createAutoBuilder(drive);
        ComplexWidgetBuilder.create(autoChooser, "Auto Chooser", "Misc")
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(1, 3);

        autoChooser.addOption("NothingAuto", new InstantCommand());
        autoChooser.addOption("VisionTest", Autos.testVision(drive, vision));
        addTuningAuto(drive);
        addAutos(drive, elevator, carriage, vision);
    }
    
    public  Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static void addTuningAuto(SwerveDrive drive){
        autoChooser.addOption("BackTest", TuningAutos.backTest(drive));
        autoChooser.addOption("ForwardTest", TuningAutos.forwardTest(drive));
        autoChooser.addOption("TurnTest", TuningAutos.turnTest(drive));
        autoChooser.setDefaultOption("SplineTest", TuningAutos.splineTest(drive));
        autoChooser.addOption("StrafeRight", TuningAutos.strafeRight(drive));
        autoChooser.addOption("StrafeLeft", TuningAutos.strafeLeft(drive));
        // // autoChooser.addOption("ForwardAndBack", TuningAutos.forwardAndBackTest(drive));
    }

    public static void addAutos(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision){
        autoChooser.addOption("left", Autos.leftOnePlusTwo(drive, elevator, carriage, vision));
        autoChooser.addOption("middleProcessor", Autos.middleProcessor(drive, elevator, carriage, vision));
        autoChooser.addOption("middleBarge", Autos.middleBarge(drive, elevator, carriage, vision));
        autoChooser.addOption("right", Autos.rightOnePlusTwo(drive, elevator, carriage, vision));
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
