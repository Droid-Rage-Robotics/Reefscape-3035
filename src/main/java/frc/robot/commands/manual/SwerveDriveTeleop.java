package frc.robot.commands.manual;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveDriveConstants;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveDriveConstants.Speed;
import frc.robot.subsystems.drive.SwerveDrive.TippingState;

public class SwerveDriveTeleop extends Command {
    private final SwerveDrive drive;
    private final Supplier<Double> x, y, turn;
    private volatile double xSpeed, ySpeed, turnSpeed;
    private Rotation2d heading;
    private static final PIDController antiTipY = new PIDController(0.006, 0, 0.0005);
    private static final PIDController antiTipX = new PIDController(0.006, 0, 0.0005);

    public SwerveDriveTeleop(SwerveDrive drive, CommandXboxController driver) {
        this.drive = drive;
        this.x = driver::getLeftX;
        this.y = driver::getLeftY;
        this.turn = driver::getRightX;
        antiTipX.setTolerance(25);
        antiTipY.setTolerance(25);

        driver.rightBumper().whileTrue(drive.setSpeed(Speed.SLOW))
            .whileFalse(drive.setSpeed(Speed.NORMAL));

        driver.b().onTrue(drive.setYawCommand(0));
        
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        xSpeed = -y.get();
        ySpeed = -x.get();
        turnSpeed = -turn.get();


        // Square inputs
        if (drive.isSquaredInputs()) {
            xSpeed = DroidRageConstants.squareInput(xSpeed);
            ySpeed = DroidRageConstants.squareInput(ySpeed);
            turnSpeed = DroidRageConstants.squareInput(turnSpeed);
        }

        // Apply Field Oriented
        if (drive.isFieldOriented()) {
            double modifiedXSpeed = xSpeed;
            double modifiedYSpeed = ySpeed;

            
            heading = drive.getRotation2d();

            modifiedXSpeed = xSpeed * heading.getCos() + ySpeed * heading.getSin();
            modifiedYSpeed = -xSpeed * heading.getSin() + ySpeed * heading.getCos();

            xSpeed = modifiedXSpeed;
            ySpeed = modifiedYSpeed;
        }

        double xTilt = drive.getRoll(); //Is this Roll or pitch
        double yTilt = drive.getPitch();// Is this Roll or pitch

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
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND * 
            drive.getTranslationalSpeed();
        ySpeed = 
            ySpeed *
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND *
            drive.getTranslationalSpeed();
        turnSpeed = 
            turnSpeed *
            SwerveDriveConstants.SwerveDriveConfig.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND.get() * 
            drive.getAngularSpeed();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        SwerveModuleState[] states = SwerveDrive.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        drive.setModuleStates(states);
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
