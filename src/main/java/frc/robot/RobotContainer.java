package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SysID.DriveSysID;
import frc.robot.SysID.SysID;
import frc.robot.commands.IntakeElementInCommand;
import frc.robot.commands.manual.ManualElevator;
import frc.robot.commands.manual.SwerveDriveTeleop;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Carriage.CarriageValue;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

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
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
		driver.a().onTrue(new InstantCommand(()->drive.resetOdometry(vision.getPose())));
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


}
