package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autos.AutoChooser;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.carriage.Arm;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Intake;
import frc.robot.subsystems.carriage.Pivot;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveDriveConstants;
import frc.robot.subsystems.drive.Telemetry;
import frc.robot.subsystems.drive.SwerveDrive.Routine;
import frc.robot.subsystems.vision.Vision;
import frc.utility.shuffleboard.ShuffleboardValue;

public class Robot extends TimedRobot {
    private final SwerveDrive drive = SwerveConfig.createDrivetrain(true);
    private final Elevator elevator = new Elevator(true);
    private final Carriage carriage = new Carriage(
        new Arm(true),
        new Pivot(true),
        new Intake(true) 
    );
    
    private final Climb climb = new Climb(false);
    private final Vision vision = new Vision();

    private final CommandXboxController driver =
		new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
	
	private final CommandXboxController operator =		
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);
    // private final CycleTracker cycleTracker = new CycleTracker();
    // private final Light light = new Light();*

    // private final DriveSysID driveSysID = new DriveSysID(drive.getSwerveModules(), drive);
    // private final SysID sysID = new SysID(pivot.getMotor(), pivot, Measurement.ANGLE);
    private Field2d field = new Field2d();
    private Telemetry telemetry = new Telemetry(SwerveDriveConstants.SwerveDriveConfig.MAX_SPEED_METERS_PER_SECOND.getValue());

    private final RobotContainer robotContainer = new RobotContainer(driver, operator);
    private final AutoChooser autoChooser = new AutoChooser(drive, elevator, carriage, vision);
    private static final Alert batteryAlert = new Alert("Battery Voltage", AlertType.kWarning);
    // public boolean teleopRan;
    private ShuffleboardValue<Double> matchTime = ShuffleboardValue.create
		(0.0, "Match Time", "Misc")
		.withWidget(BuiltInWidgets.kTextView)
		.build();   
    private Command autonomousCommand;
  
    @Override
    public void robotInit() {
        // vision.setUpVision();
        // teleopRan = false;
        // CameraServer.startAutomaticCapture(); //DO NOT USE
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        field.setRobotPose(drive.getPose());
        SmartDashboard.putData("DrivePose",field);
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

        if(RobotController.getBatteryVoltage()<12.5){
            batteryAlert.set(true);
            batteryAlert.setText("Battery Voltage Low");

            // light.setAllColor(light.batteryBlue);
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
        autonomousCommand = autoChooser.getAutonomousCommand();
        // autonomousCommand = new InstantCommand();

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

        robotContainer.testNewDrive(drive, elevator);
        // if (autonomousCommand != null) {
        //     autonomousCommand.cancel();
        // }
		DriverStation.silenceJoystickConnectionWarning(true);
        // drive.changeAllianceRotation();
        // robotContainer.configureTeleOpBindings(drive, elevator, carriage, climb, vision);
        // robotContainer.resetClimb(climb);
        // vision.setUpVision(); //Has to be here to set up Limelight Pipelines

        // robotContainer.driveSysID(drive, Routine.STEER);
        // robotContainer.sysID(sysID);
        // teleopRan = true;
    }

    @Override
    public void teleopPeriodic() {
        matchTime.set(DriverStation.getMatchTime());

        // while(true){
		// 	// new OperatorXboxControllerRumble(driver, RumbleType.kBothRumble, 2, 1);
   		// 	driver.getHID().setRumble(RumbleType.kBothRumble, 0);
		// } 
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
    }

        
}