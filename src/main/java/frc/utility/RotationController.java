package frc.utility;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.drive.SwerveDrive;

/**
 * Uses a profiled PID Controller to quickly turn the robot to a specified angle. Once the robot is
 * within a certain tolerance of the goal angle, a PID controller is used to hold the robot at that
 * angle.
 */
public class RotationController implements Sendable {
    private final SwerveDrive drive;
    private final PIDController controller;
    private final PIDController holdController;

    private final Supplier<Double> headingRadians;

    private final AtomicReference<Double> calculation = new AtomicReference<Double>(0.0);
    private final AtomicReference<Double> holdCalculation = new AtomicReference<Double>(0.0);

    // Constraints constraints;

    // @AutoLogOutput(key = "Swerve/RotationController/Output")
    
    
    double calculatedValue = 0;

    double feedbackSetpoint;
    double tolerance = (Math.PI / 720);
    
    

    public RotationController(SwerveDrive drive, Supplier<Rotation2d> rotation2d) {
        this.drive=drive;

        headingRadians = () -> rotation2d.get().getRadians();

        // config = swerve.config;
        // constraints = new Constraints(config.maxAngularVelocity, config.maxAngularAcceleration);
        controller = new PIDController(0,0,0);
        

        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(tolerance * 2);

        // These are currently magic number and need to be put into SwerveConfig
        holdController = new PIDController(10, 0,0); // these probably have to be found again; most likely why robot
        // rotation is slightly oscillating in heading lock

        holdController.enableContinuousInput(-Math.PI, Math.PI);
        holdController.setTolerance(tolerance);

        calculatedValue = 0;
        feedbackSetpoint = 0.35;
    }

    public double calculate(double goalRadians) {
        // calculatedValue = ;
        calculation.set(controller.calculate(headingRadians.get(), goalRadians));
        // RobotTelemetry.print(
        //         "RotationControllerOutput: "
        //                 + calculatedValue
        //                 + " Measure: "
        //                 + measurement
        //                 + " Goal: "
        //                 + goalRadians
        //                 + " max: "
        //                 + config.maxAngularVelocity);
        if (atSetpoint()) {
            return calculatedValue = 0; // calculateHold(goalRadians);
        } else {
            return calculatedValue;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(null, calculation::get, null);
        // builder 
    }

    public double calculateHold(double goalRadians) {
        holdCalculation.set(holdController.calculate(drive.getRotation2d().getRadians(), goalRadians));

        return calculatedValue;
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public boolean atFeedbackSetpoint() {
        return Math.abs(calculatedValue) <= feedbackSetpoint;
    }

    public boolean atHoldSetpoint() {
        return holdController.atSetpoint();
    }

    public void reset() {
        // controller.
        // controller.reset(drive.getRotation2d().getRadians());
        holdController.reset();
    }

    public void updatePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }

    

    // public void setLaunchPID() {
    //     controller.setPID(
    //             launchingKPRotationController,
    //             launchingKIRotationController,
    //             launchingKDRotationController);
    // }

    // public void setConfigPID() {
    //     controller.setPID(
    //             config.kPRotationController,
    //             config.kIRotationController,
    //             config.kDRotationController);
    // }
}