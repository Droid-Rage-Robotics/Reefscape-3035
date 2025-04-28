package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import frc.utility.motor.TalonEx;

public class SwerveConfig {
    public static class Constants {
        private static final Distance WHEEL_RADIUS = Inches.of(2);
        
        public static final Distance TRACK_WIDTH = Inches.of(28.5);
        public static final Distance WHEEL_BASE = Inches.of(28.5);
        public static final LinearVelocity PHYSICAL_MAX_SPEED = MetersPerSecond.of(4.47);
        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(PHYSICAL_MAX_SPEED.in(MetersPerSecond)/4);
        public static final AngularVelocity PHYSICAL_MAX_ANGULAR_SPEED = RadiansPerSecond.of(2 * (2 * Math.PI));
        public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(PHYSICAL_MAX_ANGULAR_SPEED.in(RadiansPerSecond)/10);
        
        private static final double DRIVE_MOTOR_GEAR_RATIO = 6.746031746031747; // 1/6.75, (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)=5.35714285714
        private static final double TURN_MOTOR_GEAR_RATIO = 21.428571428571427; // 1/ 21.42

        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 35; //50, 40
        public static final double DRIVE_STATOR_CURRENT_LIMIT = 75; //90, 80
        public static final double TURN_SUPPLY_CURRENT_LIMIT = 80;
    }
    
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(2.66).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKS(0.13).withKV(2.7); // NO CHANGE

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120.0);

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(TalonEx.withCurrentLimits(
            Constants.DRIVE_SUPPLY_CURRENT_LIMIT, 
            Constants.DRIVE_STATOR_CURRENT_LIMIT)          
        );
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(TalonEx.withCurrentLimits(
            Constants.TURN_SUPPLY_CURRENT_LIMIT)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    private static final Pigeon2Configuration pigeonConfigs = null; // Leave null

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("drive", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.73);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 13;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(Constants.DRIVE_MOTOR_GEAR_RATIO)
            .withSteerMotorGearRatio(Constants.TURN_MOTOR_GEAR_RATIO)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(Constants.WHEEL_RADIUS)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(Constants.PHYSICAL_MAX_SPEED)
            .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    private static class FrontLeft {
        private static final int driveId = 12;
        private static final int turnId = 10;
        private static final int encoderId = 11;
        private static final Angle encoderOffset = Rotations.of(0.326171875);
        private static final boolean turnInverted = true;
        private static final boolean encoderInverted = false;
        private static final Distance xPos = Inches.of(11.375);
        private static final Distance yPos = Inches.of(11.375);
    }

    private static class FrontRight {
        private static final int driveId = 3;
        private static final int turnId = 1;
        private static final int encoderId = 2;
        private static final Angle encoderOffset = Rotations.of(-0.40771484375);
        private static final boolean turnInverted = true;
        private static final boolean encoderInverted = false;
        private static final Distance xPos = Inches.of(11.375);
        private static final Distance yPos = Inches.of(-11.375);
    }

    private static class BackLeft {
        private static final int driveId = 9;
        private static final int turnId = 7;
        private static final int encoderId = 8;
        private static final Angle encoderOffset = Rotations.of(0.15185546875);
        private static final boolean turnInverted = true;
        private static final boolean encoderInverted = false;
        private static final Distance xPos = Inches.of(-11.375);
        private static final Distance yPos = Inches.of(11.375);
    }

    private static class BackRight {
        private static final int driveId = 6;
        private static final int turnId = 4;
        private static final int encoderId = 5;
        private static final Angle encoderOffset = Rotations.of(0.146728515625);
        private static final boolean turnInverted = true;
        private static final boolean encoderInverted = false;
        private static final Distance xPos = Inches.of(-11.375);
        private static final Distance yPos = Inches.of(-11.375);
    }

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeft =
        ConstantCreator.createModuleConstants(
            FrontLeft.turnId, FrontLeft.driveId, FrontLeft.encoderId, FrontLeft.encoderOffset,
            FrontLeft.xPos, FrontLeft.yPos, kInvertLeftSide, FrontLeft.turnInverted, FrontLeft.encoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRight =
        ConstantCreator.createModuleConstants(
            FrontRight.turnId, FrontRight.driveId, FrontRight.encoderId, FrontRight.encoderOffset,
            FrontRight.xPos, FrontRight.yPos, kInvertRightSide, FrontRight.turnInverted, FrontRight.encoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeft =
        ConstantCreator.createModuleConstants(
            BackLeft.turnId, BackLeft.driveId, BackLeft.encoderId, BackLeft.encoderOffset,
            BackLeft.xPos, BackLeft.yPos, kInvertLeftSide, BackLeft.turnInverted, BackLeft.encoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRight =
        ConstantCreator.createModuleConstants(
            BackRight.turnId, BackRight.driveId, BackRight.encoderId, BackRight.encoderOffset,
            BackRight.xPos, BackRight.yPos, kInvertRightSide, BackRight.turnInverted, BackRight.encoderInverted
        );

    public static SwerveDrive createDrivetrain(boolean isEnabled) {
        return new SwerveDrive(
            isEnabled, DrivetrainConstants, frontLeft, frontRight, backLeft, backRight
        );
    }

    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}
