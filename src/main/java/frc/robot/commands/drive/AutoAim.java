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
	private ProfiledPIDController turntXController, distancetYController; // Can also use a normal PID Control
	// private double distanceTarget, turnTarget;
	
    protected static final ShuffleboardValue<Double> distancetYTarget = ShuffleboardValue
        .create(0.0, "Target/distancetY", Vision.class.getSimpleName()).build();
	
	protected static final ShuffleboardValue<Double> turntXTarget = ShuffleboardValue
			.create(0.0, "Target/turtX", Vision.class.getSimpleName()).build();
	public AutoAim(SwerveDrive drive, Vision vision, CommandXboxController driver) {
	this.driver = driver;
	// this.angleGoal = angleGoal;
	turntXController = new ProfiledPIDController(	
		0.0005, //.1
		0,
		0,
		new TrapezoidProfile.Constraints(1.525, 1));
	turntXController.setTolerance(.5);//-27 degrees to 27 degrees

	distancetYController = new ProfiledPIDController(
		0.1, //.034
		0,
		0,
		new TrapezoidProfile.Constraints(1.525, 1));
	distancetYController.setTolerance(.5);

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
				distancetYTarget.set(9.);
				turntXTarget.set(2.5);
				break;
			case LEFT:
				name = DroidRageConstants.leftLimelight;
				distancetYTarget.set(5.);
				turntXTarget.set(-9.);
				break;
		}
		//-12,-8.7
		if(vision.isID(name)&& vision.gettV(name)){
			drive.drive(distancetYController.calculate(vision.gettY(name), distancetYTarget.get()), // ty=x
				0,
				-turntXController.calculate(vision.gettX(name), turntXTarget.get()));// tx = turn
		} // tA forward/back
		else{
			drive.drive(0, 0, 0);
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (turntXController.atSetpoint() && distancetYController.atSetpoint()) || !driver.povUp().getAsBoolean();
	}
}