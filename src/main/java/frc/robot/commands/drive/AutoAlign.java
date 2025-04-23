package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.old.OldSwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.utility.shuffleboard.ShuffleboardValue;

public class AutoAlign extends Command {
	private OldSwerveDrive drive;
	private Vision vision;
	// private Timer timer= new Timer();
	
    private static final ShuffleboardValue<Double> aim = 
        ShuffleboardValue.create(0.0, "Aim", Vision.class.getSimpleName()).build();
		
		private static final ShuffleboardValue<Double> range = 
        ShuffleboardValue.create(0.0, "Range", Vision.class.getSimpleName()).build();

	// Auto
	public AutoAlign(OldSwerveDrive drive, Vision vision) {
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
			case MIDDLE:
				if (!vision.gettV(DroidRageConstants.rightLimelight) &&
						(!vision.gettV(DroidRageConstants.leftLimelight))) {
					return;
				}
				break;
		}

		// if (!vision.gettV(DroidRageConstants.rightLimelight) &&
		// 		(!vision.gettV(DroidRageConstants.leftLimelight))) {
		// 	return;
		// }
		aim.set((vision.aim()));
		range.set(vision.range());
		drive.drive(vision.range(), 0, vision.aim());
		vision.isAlignWriter.set(true);
	}

	@Override
	public boolean isFinished() {
		// return !driver.povUp().getAsBoolean();
		// if(vision.rotController.atSetpoint() && vision.xController.atSetpoint()){
		// 	vision.isAlignWriter.set(false);
		// 	return true;
		// }
		return (vision.rotController.atSetpoint() && vision.xController.atSetpoint());// || timer.hasElapsed(5);
	}

}
