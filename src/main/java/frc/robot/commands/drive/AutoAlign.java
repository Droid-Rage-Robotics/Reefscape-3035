package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAlign extends Command{
	private SwerveDrive drive;
	// private Light light;
	private Vision vision;
	private Vision.Location location;

	public AutoAlign(SwerveDrive drive, Vision vision) {
		this.drive = drive;
		this.vision = vision;

		switch (DroidRageConstants.alignmentMode) {
			case RIGHT:
				location = Vision.Location.RIGHT_R;
				break;
			case LEFT:
				location = Vision.Location.LEFT_L;
				break;
		}

		addRequirements(drive, vision);
	}

	@Override
	public void execute() {
		switch (DroidRageConstants.alignmentMode) {
			case RIGHT:
				drive.drive(
					vision.rotController.calculate(vision.gettX(DroidRageConstants.rightLimelight), location.getDistance()), 
					0, 
					vision.rotController.calculate(vision.gettY(DroidRageConstants.rightLimelight), location.getAngle()) - 0.03);
				break;
			case LEFT:
				drive.drive(
					vision.rotController.calculate(vision.gettX(DroidRageConstants.leftLimelight), location.getDistance()), 
					0, 
					vision.rotController.calculate(vision.gettY(DroidRageConstants.leftLimelight), location.getAngle()) - 0.03);
				break;
		}
	}
  
	@Override
	public boolean isFinished() {
		return (vision.rotController.atSetpoint()&&vision.xController.atSetpoint());
	}
}
