package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAim extends Command {
  private SwerveDrive drive;
  // private Light light;
  private Vision vision;
  private CommandXboxController driver;
  private double angleGoal;
  private ProfiledPIDController turnController, distanceController; // Can also use a normal PID Control
  public AutoAim(SwerveDrive drive, Vision vision, CommandXboxController driver, double angleGoal) {
    this.driver = driver;
    this.angleGoal = angleGoal;
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
    distanceController.setTolerance(3);

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
    if(vision.gettV()){
      drive.drive(distanceController.calculate(vision.gettY(), -2),//ty=x
          0,
          turnController.calculate(vision.gettX(), angleGoal));//tx = turn
    }
  }

  // Returns true when the command should end.
  @Override

  public boolean isFinished() {
    // Should be Moved to LightCommand
    //   light.setAllColor(light.red);
    // if(turnController.atSetpoint()&&distanceController.atSetpoint()){
    //   light.setAllColor(light.green);
    // }
    return turnController.atSetpoint()&&distanceController.atSetpoint() || !driver.povUp().getAsBoolean();
  }
}