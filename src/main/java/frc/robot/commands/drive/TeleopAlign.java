package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.Location;
//with bumper
//L4-14 inches
//L3- 11.5
//L2- 11.523

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
					vision.gettX(DroidRageConstants.leftLimelight), getLeftLocation(DroidRageConstants.leftLimelight).getAngle());
				break;
			case RIGHT:
				targetingAngularVelocity = vision.rotController.calculate(
					vision.gettX(DroidRageConstants.rightLimelight), getRightLocation(DroidRageConstants.rightLimelight).getAngle());
				break;
		}
		return targetingAngularVelocity;//-
	}

	double range() {
		double targetingForwardSpeed =0;
		switch(DroidRageConstants.alignmentMode){
			case LEFT:
				targetingForwardSpeed = vision.xController.calculate(
					vision.gettY(DroidRageConstants.leftLimelight), getLeftLocation(DroidRageConstants.leftLimelight).getDistance());
				break;
			case RIGHT:
				targetingForwardSpeed = vision.xController.calculate(
					vision.gettY(DroidRageConstants.rightLimelight), getRightLocation(DroidRageConstants.rightLimelight).getDistance());
				break;
		}
		
		// = xController.calculate(vision.gettY(DroidRageConstants.leftLimelight), -2);
		// targetingForwardSpeed *=  SwerveDriveConstants.SwerveDriveConfig.MAX_SPEED_METERS_PER_SECOND.getValue();
		return targetingForwardSpeed;
	}

	private Location getLeftLocation(String name){
		switch(vision.getID(name)){
			case 17:
				return Vision.Location.LEFT_L_L4_17;
			case 18:
			return Vision.Location.LEFT_L_L4_18;
			case 19:
				return Vision.Location.LEFT_L_L4_19;
			case 20:
				return Vision.Location.LEFT_L_L4_20;
			case 21:
				return Vision.Location.LEFT_L_L4_21;
			case 22:
				return Vision.Location.LEFT_L_L4_22;
			
			case 6:
				return Vision.Location.LEFT_L_L4_6;
			case 7:
				return Vision.Location.LEFT_L_L4_7;
			case 8:
				return Vision.Location.LEFT_L_L4_8;
			case 9:
				return Vision.Location.LEFT_L_L4_9;
			case 10:
				return Vision.Location.LEFT_L_L4_10;
			case 11:
				return Vision.Location.LEFT_L_L4_11;

			default:
				return Vision.Location.LEFT_L_L4_17;
		}

	}
	private Location getRightLocation(String name){
		switch(vision.getID(name)){
			case 17:
				return Vision.Location.RIGHT_R_L4_17;
			case 18:
			return Vision.Location.RIGHT_R_L4_18;
			case 19:
				return Vision.Location.RIGHT_R_L4_19;
			case 20:
				return Vision.Location.RIGHT_R_L4_20;
			case 21:
				return Vision.Location.RIGHT_R_L4_21;
			case 22:
				return Vision.Location.RIGHT_R_L4_22;
			
			case 6:
				return Vision.Location.RIGHT_R_L4_6;
			case 7:
				return Vision.Location.RIGHT_R_L4_7;
			case 8:
				return Vision.Location.RIGHT_R_L4_8;
			case 9:
				return Vision.Location.RIGHT_R_L4_9;
			case 10:
				return Vision.Location.RIGHT_R_L4_10;
			case 11:
				return Vision.Location.RIGHT_R_L4_11;
			default:
				return Vision.Location.RIGHT_R_L4_17;
		}

	}

}
