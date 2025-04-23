package frc.robot.subsystems.drive.old;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDriveConstants.SwerveDriveConfig;
import frc.utility.encoder.EncoderEx.EncoderDirection;
import frc.utility.motor.CANMotorEx.Direction;
import frc.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.utility.motor.TalonEx;
import frc.utility.shuffleboard.ShuffleboardValue;
import lombok.Getter;

public class SwerveModule {
    public enum POD{
        FL,
        BL,
        FR,
        BR
    }

    public static class Constants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 1/6.75;//(50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)=5.35714285714
        public static final double TURN_MOTOR_GEAR_RATIO = 1/ 21.42;

        public static final double DRIVE_ENCODER_ROT_2_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        public static final double READINGS_PER_REVOLUTION = 1;//4096

        //Used for the CANCoder
        public static final double TURN_ENCODER_ROT_2_RAD = 2 * Math.PI / READINGS_PER_REVOLUTION;
        public static final double TURN_ENCODER_ROT_2_RAD_SEC = TURN_ENCODER_ROT_2_RAD/60;

        //0.5 Change this to make the robot turn the turn motor as fast as possible
        //If strafing, the robot drifts to he front/back, then increase
                //.115

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.47;

        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 35;//50, 40
        public static final double DRIVE_STATOR_CURRENT_LIMIT = 75;   //90, 80
        public static final double TURN_SUPPLY_CURRENT_LIMIT = 80;
    }

    @Getter private TalonEx driveMotor;

    @Getter private TalonEx turnMotor;
    // private CANcoderEx turnEncoder;
    private CANcoder turnEncoder;
    private CANcoderConfiguration config = new CANcoderConfiguration();

    private PIDController turningPidController;
    private SimpleMotorFeedforward feedforward;

    private ShuffleboardValue<Double> turnPositionWriter;
    private ShuffleboardValue<Double> drivePositionWriter;
    private String subsystemName;
    private SwerveModule.POD podName;

    // MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

    public SwerveModule(){
        
    }

    public static SwerveModule.SubsystemNameBuilder create() {
        SwerveModule module = new SwerveModule();
        return module.new SubsystemNameBuilder();
    }
    public class SubsystemNameBuilder {
        public DriveIDBuilder withSubsystemName(SubsystemBase base, SwerveModule.POD pod) {
            podName = pod;
            subsystemName = base.getClass().getSimpleName();
            turnPositionWriter = ShuffleboardValue.create(0.0, 
                "Module/Turn Position (Radians)" + podName.toString(), 
                subsystemName).build();
            drivePositionWriter = ShuffleboardValue.create(0.0, 
                "Module/Drive Position (Radians)" + podName.toString(), 
                subsystemName).build();
                return new DriveIDBuilder();
        }
        // public DriveIDBuilder withSubsystemName(SubsystemBase base, SwerveModule.POD pod) {
        //     return withSubsystemName(base.getClass().getSimpleName(), pod);
        // }
    }
    public class DriveIDBuilder {
        public TurnIDBuilder withDriveMotor(int driveMotorId, Direction driveMotorReversed, boolean isEnabled){ 
            driveMotor = TalonEx.create(driveMotorId, DroidRageConstants.driveCanBus)
                .withDirection(driveMotorReversed)
                .withIdleMode(ZeroPowerMode.Brake)
                .withPositionConversionFactor(Constants.DRIVE_ENCODER_ROT_2_METER)
                .withSubsystemName(subsystemName)
                .withIsEnabled(isEnabled)
                .withCurrentLimit(Constants.DRIVE_SUPPLY_CURRENT_LIMIT, Constants.DRIVE_STATOR_CURRENT_LIMIT);
            return new TurnIDBuilder();
        }
    }
    public class TurnIDBuilder {
        public EncoderBuilder withTurnMotor(int turnMotorId, Direction turningMotorReversed, boolean isEnabled){
            turnMotor = TalonEx.create(turnMotorId, DroidRageConstants.driveCanBus)
                .withDirection(turningMotorReversed)
                .withIdleMode(ZeroPowerMode.Coast)
                .withPositionConversionFactor(Constants.TURN_ENCODER_ROT_2_RAD)
                .withSubsystemName(subsystemName)
                .withIsEnabled(isEnabled)
                .withCurrentLimit(Constants.TURN_SUPPLY_CURRENT_LIMIT);
            return new EncoderBuilder();
        }
    }
    public class EncoderBuilder{
        @SuppressWarnings("unchecked")
        public <T extends SwerveModule> T withEncoder(int absoluteEncoderId, Supplier<Double> absoluteEncoderOffsetRad,
            EncoderDirection absoluteEncoderReversed){
                turnEncoder = new CANcoder(absoluteEncoderId, DroidRageConstants.driveCanBus);
                // config.MagnetSensor.SensorDirection = switch (absoluteEncoderReversed) {
                //     case Forward -> SensorDirectionValue.Clockwise_Positive;
                //     case Reversed -> SensorDirectionValue.CounterClockwise_Positive;
                // };
                config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                // switch (EncoderRange.ZERO_TO_ONE) {
                //     case PLUS_MINUS_HALF:
                //         config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
                //     case ZERO_TO_ONE:
                //         config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
                // }
                config.MagnetSensor.MagnetOffset = (absoluteEncoderOffsetRad.get()/Constants.TURN_ENCODER_ROT_2_RAD);
                config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
                turnEncoder.getConfigurator().apply(config);
                
            // turnEncoder = CANcoderEx.create(absoluteEncoderId, DroidRageConstants.driveCanBus)
            //     .withDirection(absoluteEncoderReversed)
            //     // .withPositionConversionFactor(1)
            //     .withOffset(absoluteEncoderOffsetRad.get()/Constants.TURN_ENCODER_ROT_2_RAD)
            //     .withSubsystemBase(podName.name(), subsystemName)
            //     .withRange(EncoderRange.ZERO_TO_ONE);
                

            turningPidController = new PIDController(SwerveDriveConfig.TURN_KP.getValue(), 0.0, 0.0);
            turningPidController.enableContinuousInput(-Math.PI, Math.PI);// Was -Math.PI, Math.PI but changed to 0 and 2PI
// 0, 2 * Math.PI
            feedforward = new SimpleMotorFeedforward(SwerveDriveConfig.DRIVE_KS.getValue(),
                    SwerveDriveConfig.DRIVE_KV.getValue());

            resetDriveEncoder();
             return (T) SwerveModule.this;
        }
    }

    public double getDrivePos() {
        drivePositionWriter.write(driveMotor.getPosition());
        return driveMotor.getPosition();
    }
    
    public double getTurningPosition() {
        turnPositionWriter.write(turnEncoder.getAbsolutePosition().getValueAsDouble()*Constants.TURN_ENCODER_ROT_2_RAD);
        return (turnEncoder.getAbsolutePosition().getValueAsDouble()*Constants.TURN_ENCODER_ROT_2_RAD);
    }

    public double getDriveVelocity(){
        // return driveMotor.getEncoder().getVelocity();
        return driveMotor.getVelocity();
    }

    public void resetDriveEncoder(){
        // driveMotor.getEncoder().setPosition(0);
        driveMotor.setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePos(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState desiredState = state;
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        desiredState.optimize(getState().angle);
        desiredState.optimize(getState().angle);
        driveMotor.setPower(state.speedMetersPerSecond / Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        turnMotor.setPower((turningPidController.calculate(getTurningPosition(), desiredState.angle.getRadians()))*1);
        // SmartDashboard.putString("Swerve[" + turnEncoder.getDeviceID() + "] state", desiredState.toString());
        // SmartDashboard.putString("Swerve[" + turnMotor.getDeviceID() + "] state", desiredState.toString());
    }

    public void setFeedforwardState(SwerveModuleState state) {
        SwerveModuleState desiredState = state;
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        desiredState.optimize(getState().angle);
        desiredState.optimize(getState().angle);
        driveMotor.setVoltage(feedforward.calculate(state.speedMetersPerSecond));
        turnMotor.setPower(turningPidController.calculate(getTurningPosition(), desiredState.angle.getRadians()));

        // SmartDashboard.putString("Swerve[" + turnEncoder.getDeviceID() + "] state", desiredState.toString());
        // SmartDashboard.putString("Swerve[" + turnMotor.getDeviceID() + "] state", desiredState.toString());
    }

    public void stop(){
        driveMotor.setPower(0);
        turnMotor.setPower(0);
    }

    public void coastMode() {
        driveMotor.setIdleMode(ZeroPowerMode.Coast);
        turnMotor.setIdleMode(ZeroPowerMode.Coast);
    }

    public void brakeMode() {
        driveMotor.setIdleMode(ZeroPowerMode.Brake);
        turnMotor.setIdleMode(ZeroPowerMode.Brake);
    }

    public void brakeAndCoastMode() {
        driveMotor.setIdleMode(ZeroPowerMode.Brake);
        turnMotor.setIdleMode(ZeroPowerMode.Coast);
    }

    public void getTurnVoltage(){
        turnMotor.getVoltage();
    }

    public void setTurnMotorIsEnabled(boolean isEnabled){
        turnMotor.setIsEnabled(isEnabled);
    }
    
    public void setDriveMotorIsEnabled(boolean isEnabled) {
        driveMotor.setIsEnabled(isEnabled);
    }
    


}