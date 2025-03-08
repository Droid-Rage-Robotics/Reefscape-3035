package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SysID.SysID;
import frc.robot.SysID.SysID.Measurement;
import frc.robot.commands.autos.AutoChooser;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.carriage.Arm;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Intake;
import frc.robot.subsystems.carriage.Pivot;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.utility.motor.CANMotorEx.Direction;
import frc.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.utility.motor.SparkMaxEx;
import frc.utility.motor.TalonEx;
import frc.utility.shuffleboard.ComplexWidgetBuilder;
import frc.utility.shuffleboard.ShuffleboardValue;

public class Robot extends TimedRobot {
    private final Vision vision = new Vision();
    private final SwerveDrive drive = new SwerveDrive(false);//-10 Works
    private final Elevator elevator = new Elevator(false);
    // // // // private Climb climb = new Climb(false);
    private Intake intake = new Intake(false);
    private Pivot pivot= new Pivot(false);
    private Arm arm = new Arm(false);
    private final Carriage carriage = new Carriage(
        arm, 
        pivot,
        intake
    );
    // private final CycleTracker cycleTracker = new CycleTracker();
    // private final Light light = new Light();

    // private final DriveSysID driveSysID = new DriveSysID(drive.getSwerveModules(), drive);
    // private final SysID sysID = new SysID(pivot.getMotor(), pivot, Measurement.ANGLE);

    private RobotContainer robotContainer = new RobotContainer();
    // private AutoChooser autoChooser = new AutoChooser(drive, vision);
    private static final Alert batteryAlert = new Alert("Battery Voltage", AlertType.kWarning);
    // public boolean teleopRan;
    private ShuffleboardValue<Double> matchTime = ShuffleboardValue.create
		(0.0, "Match Time", "Misc")
		.withWidget(BuiltInWidgets.kTextView)
		.build();
    private Command autonomousCommand;
  
    @Override
    public void robotInit() {
        // teleopRan = false;
        
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // if(DriverStation.isEStopped()){ //Robot Estopped
        //     light.flashingColors(light.red, light.white);
        // }
    }

    @Override
    public void disabledInit() {
        // if(teleopRan) {
        //     cycleTracker.printAllData(carriage);
        // }
    }
    
    @Override
    public void disabledPeriodic() {
        //In Here, Try using controller to pick the auto

        if(RobotController.getBatteryVoltage()<11.5){
            batteryAlert.set(true);

            // light.setAllColor(light.batteryBlue);
            
            // drive.playMusic(2);
        } 
        else{
            batteryAlert.set(false);
            // light.flashingColors(light.yellow, light.blue);
        }
        // light.setAllColor(light.blue);
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        // autonomousCommand = autoChooser.getAutonomousCommand();
        autonomousCommand = new InstantCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // if(DriverStation.isEStopped()){ //Robot Estopped
        //     light.flashingColors(light.red, light.white);
        // }
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        // if (autonomousCommand != null) {
        //     autonomousCommand.cancel();
        // }
		DriverStation.silenceJoystickConnectionWarning(true);
        // robotContainer.configureTeleOpBindings(drive, elevator, carriage, climb);
        // robotContainer.sysID(driveSysID);
        // robotContainer.sysID(sysID);
        robotContainer.testDrive(drive, vision);
        // robotContainer.testIntake(motor);
        // robotContainer.testMotor(motorR, motorL);
        // robotContainer.testMotor(motor);
        // robotContainer.testClimb(climb);
        // robotContainer.testElevator(elevator);

        // robotContainer.testCarriage(elevator, carriage);

        // robotContainer.testCANivore(driveMotor, motor);
        // teleopRan = true;
    }

    @Override
    public void teleopPeriodic() {
        matchTime.set(DriverStation.getMatchTime());
    }
    
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void teleopExit(){
        // cycleTracker.printAllData();
    }

    @Override
    public void autonomousExit(){
        if (autonomousCommand != null) {
        autonomousCommand.cancel();
        }
        // drive.changeAllianceRotation();
    }

        
}