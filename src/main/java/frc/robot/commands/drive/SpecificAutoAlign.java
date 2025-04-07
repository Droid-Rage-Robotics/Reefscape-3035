package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class SpecificAutoAlign extends Command {
	private SwerveDrive drive;
	private Vision vision;
	private int look;
	// private Timer timer= new Timer();

	// TELEOP
	public SpecificAutoAlign(SwerveDrive drive, Vision vision, int look) {
		this.drive = drive;
		this.vision = vision;
		this.look = look;
		addRequirements(vision);
	}

	@Override
	public void execute() {
		switch (DroidRageConstants.alignmentMode) {
			case LEFT:
				while(vision.getID(DroidRageConstants.leftLimelight)==look){
				// while(!vision.gettV(DroidRageConstants.leftLimelight)){
					drive.drive(0, 0, -0.1);
				}
				break;
			case RIGHT:
				while (vision.getID(DroidRageConstants.rightLimelight) == look) {
					// while(!vision.gettV(DroidRageConstants.rightLimelight)){
					drive.drive(0, 0, 0.1);
				}
				break;
		}
		// timer.restart();
		drive.drive(vision.rangeAuto(look), 0, vision.aimAuto(look) - 0.03);
	}

	@Override
	public boolean isFinished() {
		// return !driver.povUp().getAsBoolean();
		return (vision.rotController.atSetpoint() && vision.xController.atSetpoint());// || timer.hasElapsed(5);
	}

}
