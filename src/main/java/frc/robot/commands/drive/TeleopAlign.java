package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class TeleopAlign extends Command{
	private SwerveDrive drive;
	private Vision vision;
	private CommandXboxController driver;

	//TELEOP
	public TeleopAlign(SwerveDrive drive, Vision vision, CommandXboxController driver) {
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
		}
		drive.drive(range(), 0, aim()-0.03);
	}
  
	@Override
	public boolean isFinished() {
		return !driver.leftBumper().getAsBoolean();
	}

	double aim() {
		double targetingAngularVelocity =0;
		switch(DroidRageConstants.alignmentMode){
			case LEFT:
				targetingAngularVelocity = vision.rotController.calculate(
					vision.gettX(DroidRageConstants.leftLimelight), Vision.Location.LEFT_L_L4.getAngle());
				break;
			case RIGHT:
				targetingAngularVelocity = vision.rotController.calculate(
					vision.gettX(DroidRageConstants.rightLimelight), Vision.Location.RIGHT_R_L4.getAngle());
				break;
		}
		return targetingAngularVelocity;//-
	}

	double range() {
		double targetingForwardSpeed =0;
		switch(DroidRageConstants.alignmentMode){
			case LEFT:
				targetingForwardSpeed = vision.xController.calculate(
					vision.gettY(DroidRageConstants.leftLimelight), Vision.Location.LEFT_L_L4.getDistance());
				break;
			case RIGHT:
				targetingForwardSpeed = vision.xController.calculate(
					vision.gettY(DroidRageConstants.rightLimelight), Vision.Location.RIGHT_R_L4.getDistance());
				break;
		}
		
		// = xController.calculate(vision.gettY(DroidRageConstants.leftLimelight), -2);
		// targetingForwardSpeed *=  SwerveDriveConstants.SwerveDriveConfig.MAX_SPEED_METERS_PER_SECOND.getValue();
		return targetingForwardSpeed;
	}

}
