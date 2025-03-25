package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

//Autonomous
public class AutoAlign extends Command {
	private SwerveDrive drive;
	private Vision vision;
	private PIDController rotController = new PIDController(.07, 0, 0);
	private PIDController xController = new PIDController(.07, 0, 0);

	public AutoAlign(SwerveDrive drive, Vision vision) {
		this.drive = drive;
		this.vision = vision;
		rotController.setTolerance(.3);
		xController.setTolerance(.2);

		// addRequirements(drive, vision);
	}

	@Override
	public void execute() {
		drive.drive(limelight_range_proportional(), 0, limelight_aim_proportional());// - 0.03);
	}

	@Override
	public boolean isFinished() {
		// return !driver.povUp().getAsBoolean();
		return (vision.rotController.atSetpoint() && vision.xController.atSetpoint());// || timer.hasElapsed(3);

	}

	double limelight_range_proportional() {
		double targetingForwardSpeed = xController.calculate(
			vision.gettY(DroidRageConstants.leftLimelight), Vision.Location.LEFT_L.getDistance());
		// targetingForwardSpeed *=
		// SwerveDriveConstants.SwerveDriveConfig.MAX_SPEED_METERS_PER_SECOND.getValue();
		return targetingForwardSpeed;
	}

	double limelight_aim_proportional() {
		double targetingAngularVelocity = rotController.calculate(
			vision.gettX(DroidRageConstants.leftLimelight), Vision.Location.LEFT_L.getAngle());
		// targetingAngularVelocity *=
		// SwerveDriveConstants.SwerveDriveConfig.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED.getValue();
		return targetingAngularVelocity;// -
	}

	

}

		