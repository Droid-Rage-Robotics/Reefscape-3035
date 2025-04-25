package frc.robot.commands.manual;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorValue;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveDrive.TippingState;
import frc.robot.subsystems.drive.SwerveDriveConstants.DriveOptions;
import frc.robot.subsystems.drive.SwerveDriveConstants.Speed;

public class SwerveDriveTeleop extends Command {
    private final SwerveDrive drive;
    private final Supplier<Double> x, y, turn;

    private volatile double xSpeed, ySpeed, turnSpeed;
    private Rotation2d heading;
    private static final PIDController antiTipY = 
        new PIDController(0.006, 0, 0.0005);
    private static final PIDController antiTipX = 
        new PIDController(0.006, 0, 0.0005);

    // private SlewRateLimiter xLimiter = new SlewRateLimiter(SwerveDriveConstants.SwerveDriveConfig.MAX_ACCELERATION_UNITS_PER_SECOND.getValue());
    // private SlewRateLimiter yLimiter = new SlewRateLimiter(SwerveDriveConstants.SwerveDriveConfig.MAX_ACCELERATION_UNITS_PER_SECOND.getValue());

    private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();
    

    public SwerveDriveTeleop(
        SwerveDrive drive, 
        CommandXboxController driver, 
        Elevator elevator
    ) {
        this.drive = drive;
        this.x = driver::getLeftX;
        this.y = driver::getLeftY;
        this.turn = driver::getRightX;
        antiTipX.setTolerance(2);
        antiTipY.setTolerance(2);

        driver.rightBumper().whileTrue(drive.setSpeed(Speed.SLOW))//SLOW
            .whileFalse(drive.setSpeed(Speed.NORMAL));//NORMAL
        // driver.rightBumper().whileTrue(drive.setSpeed(Speed.SUPER_SLOW))
        //     .whileFalse(drive.setSpeed(Speed.SLOW));
        
        driver.b().onTrue(drive.setYawCommand(0));

        if(elevator.getEncoderPosition() >= ElevatorValue.L3.getHeight()){ 
            drive.setSpeed(Speed.SLOW);
        }

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        xSpeed = -y.get(); //Forward
        ySpeed = -x.get(); //Strafe
        turnSpeed = -turn.get(); //Turn


        // Square inputs
        if (DriveOptions.IS_SQUARED_INPUTS.get()) {
            xSpeed = DroidRageConstants.squareInput(xSpeed);
            ySpeed = DroidRageConstants.squareInput(ySpeed);
            turnSpeed = DroidRageConstants.squareInput(turnSpeed);
        }

        // Apply Field Oriented
        if (DriveOptions.IS_FIELD_ORIENTED.get()) {
            double modifiedXSpeed = xSpeed;
            double modifiedYSpeed = ySpeed;

            
            heading = drive.getRotation2d();
            

            modifiedXSpeed = xSpeed * heading.getCos() + ySpeed * heading.getSin();
            modifiedYSpeed = -xSpeed * heading.getSin() + ySpeed * heading.getCos();
            

            xSpeed = modifiedXSpeed;
            ySpeed = modifiedYSpeed;
        }

        
       
        

        // Apply Anti-Tip
        // double xTilt = drive.getRoll(); //Is this Roll or pitch
        // double yTilt = drive.getPitch();// Is this Roll or pitch
        double xTilt = drive.getPigeon2().getRoll().getValueAsDouble();
        double yTilt = drive.getPigeon2().getPitch().getValueAsDouble();

        if(drive.getTippingState()==TippingState.ANTI_TIP) {//Need to take into account on the direction of the tip
            if (Math.abs(xTilt) > 10)
                xSpeed = -antiTipX.calculate(xTilt, 0);
            if (Math.abs(yTilt) >10)
                ySpeed = -antiTipY.calculate(yTilt, 0);
        }

        // Apply deadzone
        if (Math.abs(xSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) xSpeed = 0;
        if (Math.abs(ySpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) ySpeed = 0;
        if (Math.abs(turnSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) turnSpeed = 0;

        // Smooth driving and apply speed
        xSpeed = 
            xSpeed *
            SwerveConfig.Constants.PHYSICAL_MAX_SPEED.in(MetersPerSecond) * 
            drive.getTranslationalSpeed();
        ySpeed = 
            ySpeed *
            SwerveConfig.Constants.PHYSICAL_MAX_SPEED.in(MetersPerSecond) *
            drive.getTranslationalSpeed();
        turnSpeed = 
            turnSpeed *
            SwerveConfig.Constants.PHYSICAL_MAX_ANGULAR_SPEED.in(RadiansPerSecond) * 
            drive.getAngularSpeed();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        // SwerveModuleState[] states = SwerveDrive.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        // drive.setModuleStates(states);

        drive.drive(driveRequest.withSpeeds(chassisSpeeds));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
