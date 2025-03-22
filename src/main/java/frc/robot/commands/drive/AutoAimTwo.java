package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAimTwo extends Command{
	private SwerveDrive drive;
	// private Light light;
	private Vision vision;
	private CommandXboxController driver;
	private PIDController rotController =new PIDController(.06,0,0);
	private PIDController xController = new PIDController(.13, 0, 0);

	public AutoAimTwo(SwerveDrive drive, Vision vision, CommandXboxController driver) {
		this.driver = driver;
		this.drive = drive;
		this.vision = vision;
		rotController.setTolerance(.3);
		xController.setTolerance(.2);

		addRequirements(drive, vision);
	}

	@Override
	public void execute() {
		switch (DroidRageConstants.alignmentMode) {
			case RIGHT:
				drive.drive(forward(DroidRageConstants.rightLimelight,0), 
				0, rotation(DroidRageConstants.rightLimelight, 0) - 0.03);
				break;
			case LEFT:
				drive.drive(forward(DroidRageConstants.rightLimelight, 0),
					0, rotation(DroidRageConstants.rightLimelight, 0) - 0.03);
				break;
		}
	}
  
	@Override
	public boolean isFinished() {
		return !driver.povUp().getAsBoolean();
	}

	private double rotation(String name, double angleGoal) {
		return rotController.calculate(vision.gettX(DroidRageConstants.leftLimelight), angleGoal);
	}

	private double forward(String name, double distanceTarget) {
		return xController.calculate(vision.gettY(DroidRageConstants.leftLimelight), distanceTarget);
	}

}
