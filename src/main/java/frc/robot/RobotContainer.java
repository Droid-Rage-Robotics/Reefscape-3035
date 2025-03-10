package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SysID.DriveSysID;
import frc.robot.SysID.SysID;
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
import frc.utility.motor.CANMotorEx;
import frc.utility.motor.SparkMaxEx;
import frc.utility.motor.TalonEx;

public class RobotContainer {
	private final CommandXboxController driver =
		new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
	
	private final CommandXboxController operator =
		new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

	public RobotContainer(){
		DriverStation.silenceJoystickConnectionWarning(true);
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

		driver.rightTrigger()
				.onTrue(carriage.setIntakeCommand(CarriageIntakeValue.INTAKE))
				// .onTrue(new CommandsList.TeleopIntakeCommand(carriage))
				.onFalse(new TeleopCommands().teleopHoldCommand(carriage));
		driver.leftTrigger()
				.onTrue(new TeleopCommands().teleopOuttakeCommand(carriage))
				.onFalse(carriage.setIntakeCommand(CarriageIntakeValue.STOP));
		// driver.rightTrigger()
		// 	.onTrue(new IntakeElementInCommand(driver, coralSubsystem));
		
			

		// operator.leftBumper()
		// 	.onTrue(elevator.setPositionCommand(Elevator.ElevatorValue.LOW))
		// 	.onTrue(coralSubsystem.setPositionCommand(CarriageValue.INTAKE_HPS));
		// operator.rightTrigger()
		// 	.onTrue(coralSubsystem.setPositionCommand(CarriageValue.INTAKE_GROUND));
		


		//coral algae
		//ground ground pickup povDown 

		//hms coral povLeft
		//    

		// l1 processor  a
		//l2  x
		//l3 low  b
		//l4 high y

		

		operator.y()
			.onTrue(carriage.setPositionCommand(CarriageValue.L4))
			.onTrue(elevator.setPositionCommand(ElevatorValue.L4));
		operator.x()
			.onTrue(carriage.setPositionCommand(CarriageValue.L3))
			.onTrue(elevator.setPositionCommand(ElevatorValue.L3));
		operator.b()
			.onTrue(carriage.setPositionCommand(CarriageValue.L2))
			.onTrue(elevator.setPositionCommand(ElevatorValue.L2));
		operator.a()
			.onTrue(carriage.setPositionCommand(CarriageValue.L1))
			.onTrue(elevator.setPositionCommand(ElevatorValue.L1));

		operator.povRight()// Coral
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_HPS))
			.onTrue(elevator.setPositionCommand(ElevatorValue.INTAKE_HPS));
		operator.povLeft()// Coral
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_HPS_BLOCK))
			.onTrue(elevator.setPositionCommand(ElevatorValue.INTAKE_HPS));
		operator.povUp()// ALgae
			.onTrue(elevator.setPositionCommand(ElevatorValue.GROUND))
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_GROUND));
		operator.povDown()
			.onTrue(elevator.setPositionCommand(ElevatorValue.GROUND))
			.onTrue(carriage.setPositionCommand(CarriageValue.HOLD));	

		operator.rightBumper()
				.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_HIGH))
				.onTrue(elevator.setPositionCommand(ElevatorValue.ALGAE_HIGH));
		operator.leftBumper()
				.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_LOW))
				.onTrue(elevator.setPositionCommand(ElevatorValue.ALGAE_LOW));
	}

	public void testDrive(SwerveDrive drive, Vision vision){
		// drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));

		driver.a().onTrue(new InstantCommand(()->drive.resetOdometry(vision.getPose())));

		driver.povUp().onTrue(new AutoAim(drive, vision, driver,7));
		driver.x().onTrue(new Turn180Degrees(drive,driver));
		// driver.povRight().onTrue(new AutoAim(drive, vision, driver, 10));
		// driver.povLeft().onTrue(new AutoAim(drive, vision, driver, -10));

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

	public void testMotor(CANMotorEx motorO, CANMotorEx motorT) {
		driver.rightTrigger()
			.onTrue(new InstantCommand(() -> motorO.setPower(-.5)))
			.onTrue(new InstantCommand(() -> motorT.setPower(-.5)))
			.onFalse(new InstantCommand(() -> motorO.setPower(0)))
			.onFalse(new InstantCommand(() -> motorT.setPower(0)));
		driver.leftTrigger()
			.onTrue(new InstantCommand(() -> motorO.setPower(.5)))
			.onTrue(new InstantCommand(() -> motorT.setPower(.5)))
			.onFalse(new InstantCommand(() -> motorO.setPower(0)))
			.onFalse(new InstantCommand(() -> motorT.setPower(0)));
		// driver.rightTrigger().whileTrue(intake.setTargetPositionCommand(10))
		// .onFalse(intake.setTargetPositionCommand(00));
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
	
	public void testMotor(TalonFX motorO) {
		driver.rightTrigger()
				.onTrue(new InstantCommand(() -> motorO.set(-.2)))
				.onFalse(new InstantCommand(() -> motorO.set(0)));
		driver.leftTrigger()
				.onTrue(new InstantCommand(() -> motorO.set(.2)))
				.onFalse(new InstantCommand(() -> motorO.set(0)));
		// driver.rightTrigger().whileTrue(intake.setTargetPositionCommand(10))
		// .onFalse(intake.setTargetPositionCommand(00));
	}

	public void testCarriage(Elevator elevator, Carriage carriage){//, TalonEx motor){
		elevator.setDefaultCommand(new ManualElevator(elevator, driver::getRightY));
		driver.povUp()//ALgae
			.onTrue(elevator.setPositionCommand(ElevatorValue.GROUND))
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_GROUND));
			
		driver.povDown()
			.onTrue(carriage.setPositionCommand(CarriageValue.HOLD));
			// .onTrue(elevator.setPositionCommand(ElevatorValue.L4));
		// driver.povDown()
			// .onTrue(carriage.setPositionCommand(CarriageValue.L2));
			// .onTrue(elevator.setPositionCommand(ElevatorValue.GROUND));
		driver.povRight()//Coral
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_HPS))
			.onTrue(elevator.setPositionCommand(ElevatorValue.INTAKE_HPS));
		driver.povLeft()// Coral
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_HPS_BLOCK))
			.onTrue(elevator.setPositionCommand(ElevatorValue.INTAKE_HPS));
			
		driver.x()
			.onTrue(carriage.setPositionCommand(CarriageValue.L4))
			.onTrue(elevator.setPositionCommand(ElevatorValue.L4));
		driver.a()
		.onTrue(carriage.setPositionCommand(CarriageValue.L3))
			.onTrue(elevator.setPositionCommand(ElevatorValue.L3));
		driver.b()
		.onTrue(carriage.setPositionCommand(CarriageValue.L2))
			.onTrue(elevator.setPositionCommand(ElevatorValue.L2));
		driver.y()
		.onTrue(carriage.setPositionCommand(CarriageValue.L1))
			.onTrue(elevator.setPositionCommand(ElevatorValue.L1));

		driver.rightBumper()
			.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_HIGH))
			.onTrue(elevator.setPositionCommand(ElevatorValue.ALGAE_HIGH));
		driver.leftBumper()
			.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_LOW))
			.onTrue(elevator.setPositionCommand(ElevatorValue.ALGAE_LOW));
			


		
		driver.rightTrigger()
			.onTrue(carriage.setIntakeCommand(CarriageIntakeValue.INTAKE))
			// .onTrue(new CommandsList.TeleopIntakeCommand(carriage))
			.onFalse(new TeleopCommands().teleopHoldCommand(carriage));
		driver.leftTrigger()
			.onTrue(new TeleopCommands().teleopOuttakeCommand(carriage))
			.onFalse(carriage.setIntakeCommand(CarriageIntakeValue.STOP));
		// driver.rightTrigger()
		// 	.onTrue(carriage.getCoralIntake().setPowerCommand(1))
		// 	.onFalse(carriage.getCoralIntake().setPowerCommand(.03));
		// driver.leftTrigger()
		// 	.onTrue(carriage.getCoralIntake().setPowerCommand(-1))
		// 	.onFalse(carriage.getCoralIntake().setPowerCommand(-.03));

		// driver.rightTrigger().onTrue(new InstantCommand(() -> motor.setPower(1)))
		// 	.onFalse(new InstantCommand(() -> motor.setPower(0.02)));
		// driver.leftTrigger().onTrue(new InstantCommand(() -> motor.setPower(-1)))
		// 	.onFalse(new InstantCommand(() -> motor.setPower(0.02)));
	}

	public void testClimb(Climb climb){
		driver.povUp()
			.onTrue(climb.setTargetPositionCommand(Climb.climb));
		driver.povDown()
			.onTrue(climb.setTargetPositionCommand(Climb.hold));	
	}
}
