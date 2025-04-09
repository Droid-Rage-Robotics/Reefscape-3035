package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.utility.shuffleboard.ShuffleboardValue;

public class AutoAlign extends Command {
	private SwerveDrive drive;
	private Vision vision;
	// private Timer timer= new Timer();
	
    private final ShuffleboardValue<Double> aim = 
        ShuffleboardValue.create(0.0, "Aim", Vision.class.getSimpleName()).build();
		
		private final ShuffleboardValue<Double> range = 
        ShuffleboardValue.create(0.0, "Range", Vision.class.getSimpleName()).build();

	// Auto
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
			case MIDDLE:
				if (!vision.gettV(DroidRageConstants.rightLimelight) &&
						(!vision.gettV(DroidRageConstants.leftLimelight))) {
					return;
				}
				break;
		}
		aim.set((vision.aim()));
		range.set(vision.range());
		drive.drive(vision.range(), 0, vision.aim());
	}

	@Override
	public boolean isFinished() {
		// return !driver.povUp().getAsBoolean();
		return (vision.rotController.atSetpoint() && vision.xController.atSetpoint());// || timer.hasElapsed(5);
	}

}
