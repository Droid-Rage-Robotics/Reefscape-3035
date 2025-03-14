package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
			.onTrue(new TeleopCommands().goL4(elevator, carriage));
		operator.x()
			.onTrue(
				new SequentialCommandGroup(
					carriage.setPositionCommand(CarriageValue.L3),
					new WaitUntilCommand(() -> carriage.getArm().atSetpoint()),
					elevator.setTargetPositionCommand(ElevatorValue.L3)));
		operator.b()
			.onTrue(
				new SequentialCommandGroup(
					carriage.setPositionCommand(CarriageValue.L2),
					new WaitUntilCommand(()->carriage.getArm().atSetpoint()),
					elevator.setTargetPositionCommand(ElevatorValue.L2)
				)
			);
			
			// .onTrue(elevator.setTargetPositionCommand(ElevatorValue.L2));
		operator.a()
			.onTrue(
				new SequentialCommandGroup(
					carriage.setPositionCommand(CarriageValue.L1),
					new WaitUntilCommand(()->carriage.getArm().atSetpoint()),
					elevator.setTargetPositionCommand(ElevatorValue.L1)
				)
			);

		operator.povRight()// Coral
			.onTrue(new TeleopCommands().intakeHPS(elevator, carriage, CarriageValue.INTAKE_HPS));
		operator.povLeft()// Coral
			.onTrue(new TeleopCommands().intakeHPS(elevator, carriage, CarriageValue.INTAKE_HPS_BLOCK));
		operator.povUp()// ALgae
			.onTrue(carriage.setPositionCommand(CarriageValue.INTAKE_GROUND))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.GROUND));
		operator.povDown()
			.onTrue(new TeleopCommands().resetCarriage(elevator, carriage));

		operator.rightBumper()
			.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_HIGH))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.ALGAE_HIGH));
		operator.leftBumper()
			.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_LOW))
			.onTrue(elevator.setTargetPositionCommand(ElevatorValue.ALGAE_LOW));
		operator.rightTrigger()
			.onTrue(new TeleopCommands().barge(elevator, carriage));
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
}
