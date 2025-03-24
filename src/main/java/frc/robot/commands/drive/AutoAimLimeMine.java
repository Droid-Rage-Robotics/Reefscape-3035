package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAimLimeMine extends Command{
	private SwerveDrive drive;
	// private Light light;
	private Vision vision;
	private CommandXboxController driver;
	private PIDController rotController =new PIDController(.06,0,0);
	private PIDController xController = new PIDController(.13, 0, 0);

	public AutoAimLimeMine(SwerveDrive drive, Vision vision, CommandXboxController driver) {
		this.driver = driver;
		this.drive = drive;
		this.vision = vision;
		rotController.setTolerance(.3);
		xController.setTolerance(.2);

		addRequirements(drive, vision);
	}

	@Override
	public void execute() {
		drive.drive(limelight_range_proportional(), 0, limelight_aim_proportional()-0.03);
	}
  
	@Override
	public boolean isFinished() {
		return !driver.povUp().getAsBoolean();
	}

	double limelight_aim_proportional() {
		double targetingAngularVelocity = rotController.calculate(vision.gettX(DroidRageConstants.leftLimelight),-8);
		// targetingAngularVelocity *=  SwerveDriveConstants.SwerveDriveConfig.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED.getValue();
		return targetingAngularVelocity;//-
	}

	double limelight_range_proportional() {
		double targetingForwardSpeed = xController.calculate(vision.gettY(DroidRageConstants.leftLimelight), -2);
		// targetingForwardSpeed *=  SwerveDriveConstants.SwerveDriveConfig.MAX_SPEED_METERS_PER_SECOND.getValue();
		return targetingForwardSpeed;
	}

}
