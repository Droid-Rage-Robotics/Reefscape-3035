package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SysID.DriveSysID;
import frc.robot.SysID.SysID;
import frc.robot.commands.TeleopIntakeCommand;
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
		SwerveDrive drive
		// Elevator elevator,
		// Carriage coralSubsystem, 
		// Climb climb
		) {
		
		// Slow Mode and Gyro Reset in the Default Command
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));

		// driver.rightTrigger()
		// 	.onTrue(new IntakeElementInCommand(driver, coralSubsystem));
		
			
		// elevator.setDefaultCommand(new ManualElevator(elevator, operator::getRightY));

		// operator.leftBumper()
		// 	.onTrue(elevator.setPositionCommand(Elevator.ElevatorValue.LOW))
		// 	.onTrue(coralSubsystem.setPositionCommand(CarriageValue.INTAKE_HPS));
		// operator.rightTrigger()
		// 	.onTrue(coralSubsystem.setPositionCommand(CarriageValue.INTAKE_GROUND));
		

		driver.b()
			.onTrue(DroidRageConstants.flipElement());

		//coral algae
		//ground ground pickup povDown 

		//hms coral povLeft
		//    

		// l1 processor  a
		//l2  x
		//l3 low  b
		//l4 high y

		

		//Button Toggle Positions
		// Coral Bindings	
		// operator.a()
		// 	.onTrue(elevator.setPositionCommand(Elevator.ElevatorValue.L1))
		// 	.onTrue(coralSubsystem.setPositionCommand(CarriageValue.L1));
		// operator.b()
		// 	.onTrue(elevator.setPositionCommand(Elevator.ElevatorValue.L2))
		// 	.onTrue(coralSubsystem.setPositionCommand(CarriageValue.L2));
		// operator.x()
		// 	.onTrue(elevator.setPositionCommand(Elevator.ElevatorValue.L3))
		// 	.onTrue(coralSubsystem.setPositionCommand(CarriageValue.L3));
		// operator.y()
		// 	.onTrue(elevator.setPositionCommand(Elevator.ElevatorValue.L4))
		// 	.onTrue(coralSubsystem.setPositionCommand(CarriageValue.L4));
		

		// Algae Bindings
		// operator.povDown()
		// 	.onTrue(elevator.setPositionCommand(Elevator.ElevatorValue.L4))
		// 	.onTrue(algaeSubsystem.setPositionCommand(AlgaeSubsystem.ArmValue.GROUND));	
		// operator.povRight()
		// 	.onTrue(elevator.setPositionCommand(Elevator.ElevatorValue.LOW))
		// 	.onTrue(algaeSubsystem.setPositionCommand(AlgaeSubsystem.ArmValue.LOW));
		// operator.povUp()
		// 	.onTrue(elevator.setPositionCommand(Elevator.ElevatorValue.HIGH))
		// 	.onTrue(algaeSubsystem.setPositionCommand(AlgaeSubsystem.ArmValue.HIGH));
		// operator.povLeft()
		// 	.onTrue(elevator.setPositionCommand(Elevator.ElevatorValue.GROUND))
		// 	.onTrue(algaeSubsystem.setPositionCommand(AlgaeSubsystem.ArmValue.PROCESSOR));	
		// operator.leftTrigger()
		// 	.onTrue(elevator.setPositionCommand(Elevator.ElevatorValue.GROUND))
		// 	.onTrue(algaeSubsystem.setPositionCommand(AlgaeSubsystem.ArmValue.SHOOT));	
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

// 	public void testCANivore(TalonEx motor, TalonEx motor2){
// 		driver.rightTrigger().whileTrue(new InstantCommand(()->motor.setPower(.4)))
// 			.onFalse(new InstantCommand(()->motor.setPower(0)));
// 			driver.leftTrigger().whileTrue(new InstantCommand(() -> motor2.setPower(.4)))
// 				.onFalse(new InstantCommand(() -> motor2.setPower(0)));
// 	}

	public void testIntake(TalonEx motor){
		driver.rightTrigger().onTrue(new InstantCommand(()->motor.setPower(.7)))
			.onFalse(new InstantCommand(()->motor.setPower(0.02)));
		driver.leftTrigger().onTrue(new InstantCommand(() -> motor.setPower(-.7)))
			.onFalse(new InstantCommand(() -> motor.setPower(0.02)));
		// driver.rightTrigger().whileTrue(intake.setTargetPositionCommand(60))
		// 	.onFalse(intake.setTargetPositionCommand(00));
	}
	
	public void testMotor(SparkMaxEx motor) {
		driver.rightTrigger()
		.whileTrue(new InstantCommand(() -> motor.setPower(1)))
				.onFalse(new InstantCommand(() -> motor.setPower(0)));
		driver.leftTrigger()
		.whileTrue(new InstantCommand(() -> motor.setPower(-1)))
				.onFalse(new InstantCommand(() -> motor.setPower(0)));
		// driver.rightTrigger().whileTrue(intake.setTargetPositionCommand(10))
		// .onFalse(intake.setTargetPositionCommand(00));
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

	// public void testElevator(Elevator elevator) {
	// 	driver.povUp()
	// 		.onTrue(elevator.setPositionCommand(ElevatorValue.L1));
	// 	driver.povDown()
	// 		.onTrue(elevator.setPositionCommand(ElevatorValue.L2));
	// 	driver.povRight()
	// 		.onTrue(elevator.setPositionCommand(ElevatorValue.START));
	// }

	public void testCarriage(Elevator elevator, Carriage carriage){//, TalonEx motor){
		elevator.setDefaultCommand(new ManualElevator(elevator, driver::getRightY));
		driver.povUp()
			.onTrue(elevator.setPositionCommand(ElevatorValue.GROUND))
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_GROUND));
			
			// .onTrue(elevator.setPositionCommand(ElevatorValue.L4));
		// driver.povDown()
			// .onTrue(carriage.setPositionCommand(CarriageValue.L2));
			// .onTrue(elevator.setPositionCommand(ElevatorValue.GROUND));
		driver.povRight()
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_HPS))
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


		driver.rightTrigger()
			.onTrue(carriage.setIntakeCommand(CarriageIntakeValue.INTAKE))
			.onFalse(carriage.setIntakeCommand(CarriageIntakeValue.STOP));
		driver.leftTrigger()
			.onTrue(new TeleopIntakeCommand(carriage))
			// .onTrue(new CommandsList.TeleopIntakeCommand(carriage))
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
