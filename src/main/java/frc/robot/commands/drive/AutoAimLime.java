package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveDriveConstants;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.Vision;

public class AutoAimLime extends Command{
	private SwerveDrive drive;
	// private Light light;
	private Vision vision;
	private CommandXboxController driver;

	public AutoAimLime(SwerveDrive drive, Vision vision, CommandXboxController driver) {
		this.driver = driver;
		this.drive = drive;
		// this.light = light;
		this.vision = vision;
		addRequirements(drive, vision);

	}

  @Override
  public void execute() {

	// while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
	// if(m_controller.getAButton())
	// {
		final var rot_limelight = limelight_aim_proportional();
		double rot = rot_limelight;

		final var forward_limelight = limelight_range_proportional();
		double xSpeed = forward_limelight;

		//while using Limelight, turn off field-relative driving.
		// fieldRelative = false;
	// }

	drive.drive(xSpeed, 0, rot);
  }
  
  @Override
  public boolean isFinished() {
	  return !driver.povUp().getAsBoolean();
  }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is
  // proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional
  // to the
  // "tx" value from the Limelight.
  double limelight_aim_proportional() {
	  // kP (constant of proportionality)
	  // this is a hand-tuned number that determines the aggressiveness of our
	  // proportional control loop
	  // if it is too high, the robot will oscillate around.
	  // if it is too low, the robot will never reach its target
	  // if the robot never turns in the correct direction, kP should be inverted.
	  double kP = .05;

	  // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
	  // rightmost edge of
	  // your limelight 3 feed, tx should return roughly 31 degrees.
	  double targetingAngularVelocity = (vision.gettX(DroidRageConstants.leftLimelight)-2) * kP;

	  // convert to radians per second for our drive method
	  targetingAngularVelocity *=  SwerveDriveConstants.SwerveDriveConfig.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED.getValue();
	  // .kMaxAngularSpeed;

	  // invert since tx is positive when the target is to the right of the crosshair
	  return -targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are
  // different.
  // if your limelight and target are mounted at the same or similar heights, use
  // "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional() {
	  double kP = .1;
	  double targetingForwardSpeed = vision.gettY(DroidRageConstants.leftLimelight) * kP;
	  targetingForwardSpeed *=  SwerveDriveConstants.SwerveDriveConfig.MAX_SPEED_METERS_PER_SECOND.getValue();
	  // Drivetrain.kMaxSpeed;
	  targetingForwardSpeed *= -1.0;
	  return targetingForwardSpeed;
  }

}
