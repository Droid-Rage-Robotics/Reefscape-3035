package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class TeleopAlign extends Command{
	private SwerveDrive drive;
	// private Light light;
	private Vision vision;
	private CommandXboxController driver;
	private Vision.Location location;

	public TeleopAlign(SwerveDrive drive, Vision vision, CommandXboxController driver, Vision.Location location) {
		this.driver = driver;
		this.drive = drive;
		this.vision = vision;
		this.location = location;

		addRequirements(drive, vision);
	}

	@Override
	public void execute() {
		switch (DroidRageConstants.alignmentMode) {
			case RIGHT:
			
				drive.drive(
					vision.rotController.calculate(vision.gettX(DroidRageConstants.leftLimelight), location.getDistance()), 
					0, 
					vision.rotController.calculate(vision.gettY(DroidRageConstants.leftLimelight), location.getAngle()) - 0.03);
				// drive.drive(
				// 	vision.rotController.calculate(vision.gettX(DroidRageConstants.rightLimelight), location.getDistance()), 
				// 	0, 
				// 	vision.rotController.calculate(vision.gettY(DroidRageConstants.rightLimelight), location.getAngle()) - 0.03);
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
		// return !driver.povUp().getAsBoolean();
		return !driver.povUp().getAsBoolean()||(vision.rotController.atSetpoint()&&vision.xController.atSetpoint());
	}
}
