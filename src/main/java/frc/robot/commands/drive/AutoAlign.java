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
		drive.drive(vision.range(), 0, vision.aim() - 0.03);
	}

	@Override
	public boolean isFinished() {
		// return !driver.povUp().getAsBoolean();
		return (vision.rotController.atSetpoint() && vision.xController.atSetpoint());// || timer.hasElapsed(5);
	}

}
