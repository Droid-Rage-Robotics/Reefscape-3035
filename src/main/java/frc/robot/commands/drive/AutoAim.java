package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.utility.shuffleboard.ShuffleboardValue;

public class AutoAim extends Command {
	private SwerveDrive drive;
	// private Light light;
	private Vision vision;
	private CommandXboxController driver;
	// private double angleGoal;
	private String name;
	private ProfiledPIDController turnController, distanceController; // Can also use a normal PID Control
	// private double distanceTarget, turnTarget;
	
    protected final ShuffleboardValue<Double> distanceTarget = ShuffleboardValue
        .create(0.0, "Target/distance", Vision.class.getSimpleName()).build();
	
	protected final ShuffleboardValue<Double> turnTarget = ShuffleboardValue
			.create(0.0, "Target/turn", Vision.class.getSimpleName()).build();
	public AutoAim(SwerveDrive drive, Vision vision, CommandXboxController driver) {
	this.driver = driver;
	// this.angleGoal = angleGoal;
	turnController = new ProfiledPIDController(
		0.09, //.1
		0,
		0,
		new TrapezoidProfile.Constraints(1.525, 1));
	turnController.setTolerance(.5);//-27 degrees to 27 degrees

	distanceController = new ProfiledPIDController(
		0.1, //.034
		0,
		0,
		new TrapezoidProfile.Constraints(1.525, 1));
	distanceController.setTolerance(.5);

	addRequirements(drive, vision);
	this.drive = drive;
	// this.light = light;
	this.vision = vision;
	}

	@Override
	public void initialize() {
		// System.out.println("AutoAiming Start");
	}

	@Override
	public void execute(){
		switch(DroidRageConstants.alignmentMode){
			case RIGHT:
				name = DroidRageConstants.rightLimelight;
				distanceTarget.set(-6.0);
				turnTarget.set(-3.0);
				break;
			case LEFT:
				name = DroidRageConstants.leftLimelight;
				distanceTarget.set(-12.0);
				turnTarget.set(-8.7);
				break;
		}
		//-12,-8.7
		if(vision.isID(name)&& vision.gettV(name)){
			drive.drive(distanceController.calculate(vision.gettY(name), distanceTarget.get()), // ty=x
				0,
				turnController.calculate(vision.gettX(name), turnTarget.get()));// tx = turn
		} // tA forward/back
		else{
			drive.drive(0, 0, 0);
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (turnController.atSetpoint() && distanceController.atSetpoint()) || !driver.povUp().getAsBoolean();
	}
}