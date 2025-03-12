package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SysID.DriveSysID;
import frc.robot.SysID.SysID;
import frc.robot.commands.IntakeElementInCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.TeleopCommands;
import frc.robot.commands.Turn180Degrees;
import frc.robot.commands.drive.AutoAim;
import frc.robot.commands.manual.ManualElevator;
import frc.robot.commands.manual.SwerveDriveTeleop;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorValue;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Carriage.CarriageIntakeValue;
import frc.robot.subsystems.carriage.Carriage.CarriageValue;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.utility.motor.TalonEx;

public class RobotContainer {
	private final CommandXboxController driver, operator;
	
	public RobotContainer(CommandXboxController driver, CommandXboxController operator){
		DriverStation.silenceJoystickConnectionWarning(true);
		this.driver = driver;
		this.operator = operator;
	}

	public void configureTeleOpBindings(
		SwerveDrive drive,
		Elevator elevator,
		Carriage carriage, 
		Climb climb
		) {
		
		// Slow Mode and Gyro Reset in the Default Command
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver, elevator));
		elevator.setDefaultCommand(new ManualElevator(elevator, operator::getRightY));
		// carriage.getCoralIntake().setDefaultCommand(new RumbleCommand(elevator, carriage, driver, operator));

		driver.rightTrigger()
				.onTrue(carriage.setIntakeCommand(CarriageIntakeValue.INTAKE))
				// .onTrue(new CommandsList.TeleopIntakeCommand(carriage))
				.onFalse(new TeleopCommands().teleopHoldCommand(carriage));
		driver.leftTrigger()
				.onTrue(new TeleopCommands().teleopOuttakeCommand(carriage))
				.onFalse(carriage.setIntakeCommand(CarriageIntakeValue.STOP));

		driver.leftBumper()
			.whileTrue(climb.setTargetPositionCommand(Climb.hold))
			.onFalse(climb.setTargetPositionCommand(Climb.climb));

		operator.y()
			.onTrue(carriage.setPositionCommand(CarriageValue.L4))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.L4));
		operator.x()
			.onTrue(carriage.setPositionCommand(CarriageValue.L3))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.L3));
		operator.b()
			.onTrue(carriage.setPositionCommand(CarriageValue.L2))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.L2));
		operator.a()
			.onTrue(carriage.setPositionCommand(CarriageValue.L1))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.L1));

		operator.povRight()// Coral
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_HPS))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.INTAKE_HPS));
		operator.povLeft()// Coral
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_HPS_BLOCK))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.INTAKE_HPS));
		operator.povUp()// ALgae
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_GROUND))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.GROUND));
		operator.povDown()
			.onTrue(carriage.setPositionCommand(CarriageValue.HOLD))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.GROUND));

		operator.rightBumper()
			.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_HIGH))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.ALGAE_HIGH));
		operator.leftBumper()
			.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_LOW))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.ALGAE_LOW));
	}

	public void testDrive(SwerveDrive drive, Vision vision){
		// drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));

		driver.a().onTrue(new InstantCommand(()->drive.resetOdometry(vision.getPose())));

		driver.povUp().onTrue(new AutoAim(drive, vision, driver,7));
		driver.x().onTrue(new Turn180Degrees(drive,driver));

	}

	public void driveSysID(DriveSysID sysID){
		driver.povUp().whileTrue(sysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		driver.povDown().whileTrue(sysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		driver.povLeft().whileTrue(sysID.sysIdDynamic(SysIdRoutine.Direction.kForward));
		driver.povRight().whileTrue(sysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));

	}

	public void sysID(SysID sysID){
		driver.povUp().whileTrue(sysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		driver.povDown().whileTrue(sysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		driver.povLeft().whileTrue(sysID.sysIdDynamic(SysIdRoutine.Direction.kForward));
		driver.povRight().whileTrue(sysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));

	}

	public void testMotor(TalonEx motorO) {
		driver.rightTrigger()
				.onTrue(new InstantCommand(() -> motorO.setPower(.01)))
				.onFalse(new InstantCommand(() -> motorO.setPower(0)));
		driver.leftTrigger()
				.onTrue(new InstantCommand(() -> motorO.setPower(-.01)))
				.onFalse(new InstantCommand(() -> motorO.setPower(0)));
		// driver.rightTrigger().whileTrue(intake.setTargetPositionCommand(10))
		// .onFalse(intake.setTargetPositionCommand(00));

		motorO.testTemp(100, 2, 0);
		// if(motorO.getTemp() > 100){
		// 	motorO.setSupplyCurrentLimit(motorO.supplyCurrentLimit-2);
		// }

	}

	public void testCarriage(Elevator elevator, Carriage carriage){//, TalonEx motor){
		elevator.setDefaultCommand(new ManualElevator(elevator, driver::getRightY));
		driver.povUp()//ALgae
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.GROUND))
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_GROUND));
			
		driver.povDown()
			.onTrue(carriage.setPositionCommand(CarriageValue.HOLD));
		driver.povRight()//Coral
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_HPS))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.INTAKE_HPS));
		driver.povLeft()// Coral
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_HPS_BLOCK))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.INTAKE_HPS));
			
		driver.x()
			.onTrue(carriage.setPositionCommand(CarriageValue.L4))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.L4));
		driver.a()
		.onTrue(carriage.setPositionCommand(CarriageValue.L3))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.L3));
		driver.b()
		.onTrue(carriage.setPositionCommand(CarriageValue.L2))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.L2));
		driver.y()
		.onTrue(carriage.setPositionCommand(CarriageValue.L1))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.L1));

		driver.rightBumper()
			.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_HIGH))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.ALGAE_HIGH));
		driver.leftBumper()
			.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_LOW))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.ALGAE_LOW));
			


		
		driver.rightTrigger()
			.onTrue(carriage.setIntakeCommand(CarriageIntakeValue.INTAKE))
			// .onTrue(new CommandsList.TeleopIntakeCommand(carriage))
			.onFalse(new TeleopCommands().teleopHoldCommand(carriage));
		driver.leftTrigger()
			.onTrue(new TeleopCommands().teleopOuttakeCommand(carriage))
			.onFalse(carriage.setIntakeCommand(CarriageIntakeValue.STOP));
	}

	public void testClimb(Climb climb){
		driver.povUp()
			.onTrue(climb.setTargetPositionCommand(Climb.climb));
		driver.povDown()
			.onTrue(climb.setTargetPositionCommand(Climb.hold));	
	}
}
