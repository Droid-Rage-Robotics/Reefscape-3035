package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAimLimeMineIMU extends Command{
	private SwerveDrive drive;
	// private Light light;
	private Vision vision;
	private CommandXboxController driver;
	private PIDController rotController =new PIDController(.06,0,0);
	private PIDController strafeController = new PIDController(.02, 0, 0);

	private PIDController xController = new PIDController(.13, 0, 0);

	//NO WORK
	public AutoAimLimeMineIMU(SwerveDrive drive, Vision vision, CommandXboxController driver) {
		this.driver = driver;
		this.drive = drive;
		this.vision = vision;
		rotController.setTolerance(.3);
		xController.setTolerance(.2);

		addRequirements(drive, vision);
	}

	@Override
	public void execute() {
		if(vision.isID(DroidRageConstants.leftLimelight)){
			drive.drive(limelight_range_proportional(), strafe(), limelight_aim_proportional() - 0.03);

		}
	}
  
	@Override
	public boolean isFinished() {
		return !driver.povUp().getAsBoolean();
	}

	double limelight_aim_proportional() {
		
		double targetingAngularVelocity = rotController.calculate(drive.getHeading(), 0);
		// targetingAngularVelocity *=
		// SwerveDriveConstants.SwerveDriveConfig.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED.getValue();
		return targetingAngularVelocity;// -
		// double targetingAngularVelocity = rotController.calculate(vision.gettX(DroidRageConstants.leftLimelight),2);
		// // targetingAngularVelocity *=  SwerveDriveConstants.SwerveDriveConfig.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED.getValue();
		// return targetingAngularVelocity;//-
	}

	double strafe() {

		double strafe = strafeController.calculate(vision.gettX(DroidRageConstants.leftLimelight), 6);
		// targetingAngularVelocity *=
		// SwerveDriveConstants.SwerveDriveConfig.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED.getValue();
		return strafe;
	}

	double limelight_range_proportional() {
		double targetingForwardSpeed = xController.calculate(vision.gettY(DroidRageConstants.leftLimelight), 0);
		// targetingForwardSpeed *=  SwerveDriveConstants.SwerveDriveConfig.MAX_SPEED_METERS_PER_SECOND.getValue();
		return targetingForwardSpeed;
	}

}
