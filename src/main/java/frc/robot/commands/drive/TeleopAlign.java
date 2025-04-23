package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.old.OldSwerveDrive;
import frc.robot.subsystems.vision.Vision;
//with bumper
//L4-14 inches
//L3- 11.5
//L2- 11.523

public class TeleopAlign extends Command{
	private OldSwerveDrive drive;
	private Vision vision;
	private CommandXboxController driver;

	//TELEOP
	public TeleopAlign(OldSwerveDrive drive, Vision vision, CommandXboxController driver) {
		this.driver = driver;
		this.drive = drive;
		this.vision = vision;
		addRequirements(drive, vision);
	}

	@Override
	public void execute() {
		switch (DroidRageConstants.alignmentMode) {
			case LEFT:
				if(!vision.gettV(DroidRageConstants.leftLimelight)){
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
		drive.drive(vision.range(), 0, vision.aim());
	}
  
	@Override
	public boolean isFinished() {
		return !driver.leftBumper().getAsBoolean();
	}

	

	
	

}
