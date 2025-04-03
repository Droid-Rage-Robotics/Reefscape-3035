package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAlign extends Command {
	private SwerveDrive drive;
	private Vision vision;
	// private Timer timer= new Timer();

	// TELEOP
	public AutoAlign(SwerveDrive drive, Vision vision) {
		this.drive = drive;
		this.vision = vision;
		addRequirements(vision);
	}

	@Override
	public void execute() {
		switch (DroidRageConstants.alignmentMode) {
			case LEFT:
				if (!vision.gettV(DroidRageConstants.leftLimelight)) {
					return;
				}
				break;
			case RIGHT:
				if (!vision.gettV(DroidRageConstants.rightLimelight)) {
					return;
				}
				break;
		}
		// timer.restart();
		drive.drive(range(), 0, aim() - 0.03);
	}

	@Override
	public boolean isFinished() {
		// return !driver.povUp().getAsBoolean();
		return (vision.rotController.atSetpoint() && vision.xController.atSetpoint());// || timer.hasElapsed(5);
	}

	double aim() {
		double targetingAngularVelocity = 0;
		switch (DroidRageConstants.alignmentMode) {
			case LEFT:
				targetingAngularVelocity = vision.rotController.calculate(
						vision.gettX(DroidRageConstants.leftLimelight), Vision.Location.LEFT_L_L4_17.getAngle());
				break;
			case RIGHT:
				targetingAngularVelocity = vision.rotController.calculate(
						vision.gettX(DroidRageConstants.rightLimelight), Vision.Location.RIGHT_R_L4_17.getAngle());
				break;
		}
		return targetingAngularVelocity;// -
	}

	double range() {
		double targetingForwardSpeed = 0;
		switch (DroidRageConstants.alignmentMode) {
			case LEFT:
				targetingForwardSpeed = vision.xController.calculate(
						vision.gettY(DroidRageConstants.leftLimelight), Vision.Location.LEFT_L_L4_17.getDistance());
				break;
			case RIGHT:
				targetingForwardSpeed = vision.xController.calculate(
						vision.gettY(DroidRageConstants.rightLimelight), Vision.Location.RIGHT_R_L4_17.getDistance());
				break;
		}

		// = xController.calculate(vision.gettY(DroidRageConstants.leftLimelight), -2);
		// targetingForwardSpeed *=
		// SwerveDriveConstants.SwerveDriveConfig.MAX_SPEED_METERS_PER_SECOND.getValue();
		return targetingForwardSpeed;
	}

}
