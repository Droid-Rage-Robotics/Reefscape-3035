package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DroidRageConstants;
import frc.robot.SysID.DriveSysID;
import frc.robot.subsystems.drive.SwerveDriveConstants.Speed;
import frc.robot.subsystems.drive.SwerveDriveConstants.SwerveDriveConfig;
import frc.robot.subsystems.drive.SwerveModule.POD;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.encoder.EncoderEx.EncoderDirection;
import frc.utility.motor.CANMotorEx.Direction;
import lombok.Getter;

//Set Voltage instead of set Power
//Set them to 90 to 100%
public class SwerveDrive extends SubsystemBase implements Dashboard {
    public enum TippingState {
        NO_TIP_CORRECTION,
        ANTI_TIP,
        ;
    }
    // Translation2d(x,y) == Translation2d(front, left)
    //front +; back -
    //left
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(SwerveDriveConfig.WHEEL_BASE.getValue() / 2,
                    SwerveDriveConfig.TRACK_WIDTH.getValue() / 2), // Front Left ++
            new Translation2d(SwerveDriveConfig.WHEEL_BASE.getValue() / 2,
                    -SwerveDriveConfig.TRACK_WIDTH.getValue() / 2), // Front Right +-
            new Translation2d(-SwerveDriveConfig.WHEEL_BASE.getValue() / 2,
                    SwerveDriveConfig.TRACK_WIDTH.getValue() / 2), // Back Left -+
            new Translation2d(-SwerveDriveConfig.WHEEL_BASE.getValue() / 2,
                    -SwerveDriveConfig.TRACK_WIDTH.getValue() / 2) // Back Right --
    );
    
    private final SwerveModule frontRight = SwerveModule.create()
        .withSubsystemName(this, POD.FR)
        .withDriveMotor(3,Direction.Forward, true)
        .withTurnMotor(1, Direction.Forward, true)
        .withEncoder(2, SwerveDriveConfig.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS::getValue, 
        EncoderDirection.Forward);
        
    private final SwerveModule backRight = SwerveModule.create()
        .withSubsystemName(this, POD.BR)
        .withDriveMotor(6, Direction.Forward, true)
        .withTurnMotor(4, Direction.Forward, true)
        // .withTurnMotor(4, Direction.Reversed, true)
        .withEncoder(5, SwerveDriveConfig.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS::getValue,
        EncoderDirection.Forward);

    private final SwerveModule backLeft = SwerveModule.create()
        .withSubsystemName(this, POD.BL)
        .withDriveMotor(9, Direction.Forward, true)
        .withTurnMotor(7, Direction.Forward, true)
        .withEncoder(8, SwerveDriveConfig.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS::getValue, 
        EncoderDirection.Forward);
    
    private final SwerveModule frontLeft = SwerveModule.create()
        .withSubsystemName(this, POD.FL)
        .withDriveMotor(12, Direction.Forward, true)
        .withTurnMotor(10, Direction.Forward, true)
        .withEncoder(11, SwerveDriveConfig.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS::getValue, 
        EncoderDirection.Forward);
    
    @Getter private final SwerveModule[] swerveModules = { frontLeft, frontRight, backLeft, backRight };
    
    private DriveSysID sysId;   

    private final Pigeon2 pigeon2 = new Pigeon2(13, DroidRageConstants.driveCanBus);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry (
        DRIVE_KINEMATICS, 
        new Rotation2d(0), 
        getModulePositions()
    );

    private final SwerveDrivePoseEstimator visionOdometry = new SwerveDrivePoseEstimator(
        DRIVE_KINEMATICS, 
        getRotation2d(), 
        getModulePositions(), 
        getPose()
        );

    private volatile Speed speed = Speed.SLOW;
    private volatile TippingState tippingState = TippingState.NO_TIP_CORRECTION;

    private final Field2d field = new Field2d();
    private final Field2d visionField = new Field2d();

    private final boolean isEnabled;

    public SwerveDrive(boolean isEnabled) {
        this.isEnabled = isEnabled;
        DashboardUtils.register(this);
        
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.brakeMode();
            // swerveModule.coastMode();
            // swerveModule.brakeAndCoast^Mode();
        }

        // Pigeon Wires are facing the front of the robot
        pigeon2.getConfigurator().apply(new MountPoseConfigs());   
        // isEnabledWriter.set(isEnabled);
        for(int num = 0; num<4; num++){
            swerveModules[num].setDriveMotorIsEnabled(isEnabled);
            swerveModules[num].setTurnMotorIsEnabled(isEnabled);
        }    

    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Drive/Swerve Drive", this);
        SmartDashboard.putData("Drive/Gyro", pigeon2);
        SmartDashboard.putData("Drive/Drive Pose", field);
        SmartDashboard.putData("Drive/Vision Pose", visionField);
        SmartDashboard.putBoolean("Drive/isEnabled", isEnabled);    
    }

    @Override
    public void practiceWriters() {
        SmartDashboard.putData("Swerve Drive", encoderDebug);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getTurningPosition(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getDriveVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> frontRight.getTurningPosition(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> backLeft.getTurningPosition(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> backRight.getTurningPosition(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRight.getDriveVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> getRotation2d().getRadians(), null);
    }

    public final Sendable encoderDebug = new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Drive/Angle "+frontLeft.getPodName(), () -> frontLeft.getTurningPosition(), null);
            builder.addDoubleProperty("Drive/Angle "+frontRight.getPodName(), () -> frontRight.getTurningPosition(), null);
            builder.addDoubleProperty("Drive/Angle "+backLeft.getPodName(), () -> backLeft.getTurningPosition(), null);
            builder.addDoubleProperty("Drive/Angle "+backRight.getPodName(), () -> backRight.getTurningPosition(), null);
            builder.addDoubleProperty("Drive/Heading", () -> getHeading(), null);
            builder.addDoubleProperty("Drive/Roll", () -> getRoll(), null);
            builder.addDoubleProperty("Drive/Pitch", () -> getPitch(), null);
        }
    };
     
    @Override
    public void periodic() {
        odometry.update(
            getRotation2d(),
            getModulePositions()
        );

        field.setRobotPose(getPose());
        field.setRobotPose(getPose());
        visionOdometry.update(getRotation2d(), getModulePositions());
        visionField.setRobotPose(getVisionPose());
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public void setUpMegaTag() {
        LimelightHelpers.SetRobotOrientation(
            "limelight",
            visionOdometry.getEstimatedPosition().getRotation().getDegrees(),
            0,
            0,
            0,
            0,
            0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        boolean doRejectUpdate = false;

        // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        if(Math.abs(getRate()) > 360)
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            visionOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            visionOdometry.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public TippingState getTippingState() {
        return tippingState;
    }

    /**
     * Yaw AKA Heading in degrees
     * @return the yaw axis rotation of the bot as a double
     */
    public double getHeading() {
        // return Math.IEEEremainder(pigeon2.getYaw().getValueAsDouble(), 360);
        double yaw = pigeon2.getYaw().getValueAsDouble();

        // Normalize to [0, 360)
        yaw = ((yaw % 360) + 360) % 360;

        return yaw;
    }

    public double getPitch() {
        return Math.IEEEremainder(pigeon2.getPitch().getValueAsDouble(), 360);
    }

    public double getRoll() {
        return Math.IEEEremainder(pigeon2.getRoll().getValueAsDouble(), 360);
    }

    public double getRate() {
        return pigeon2.getAngularVelocityZWorld().getValueAsDouble();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble());
        //THe negative is supposed to help work for teleop; Should FIX

    }

    // public void setPose(Pose2d pose){
    //     odometry.resetPose(pose);
    // }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Pose2d getVisionPose() {
        return visionOdometry.getEstimatedPosition();
    }


    public double getTranslationalSpeed() {
        return speed.getTranslationalSpeed();
    }

    public double getAngularSpeed() {
        return speed.getAngularSpeed();
    }

    public double getForwardVelocity() {
        return (frontLeft.getDriveVelocity() + frontRight.getDriveVelocity()) / 2;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void drive(double xSpeed, double ySpeed, double turnSpeed) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        drive(chassisSpeeds);
    }
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = SwerveDrive.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        // if (!isEnabledWriter.get()) return;
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, 
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND
        );

        // swerveModules[1].setState(states[1]);
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setState(states[i]);
        }
    }

    public void setFeedforwardModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = SwerveDrive.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setFeedforwardModuleStates(states);
    }
    public void setFeedforwardModuleStates(SwerveModuleState[] states) {
        if (!isEnabled) return;
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, 
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND
        );

        for (int i = 0; i < 4; i++) {
            swerveModules[i].setFeedforwardState(states[i]);
        }
    }

    public void stop() {
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.stop();
        }
    }

    public void setTippingState(TippingState tippingState) {
        this.tippingState = tippingState;
        // tippingStateWriter.set(tippingState.name());
    }

    public Command setSpeed(Speed speed) {
        return runOnce(() -> {
            this.speed = speed;
            // speedStateWriter.set(speed.name());
        });
    }

    public Command resetEncoders() {
        return runOnce(() -> {
            for (SwerveModule swerveModule: swerveModules) {
                swerveModule.resetDriveEncoder();
                // pigeon2.setYaw(getAngularSpeed())
            }
        });
    }

    public Command setYawCommand(double degrees) {
        return runOnce(
            () -> setYaw(degrees)
        );
    }
    public void setYaw(double degrees){
        pigeon2.setYaw(degrees, 5);
    }

    public Command runStop() {
        return runOnce(this::stop);
    }

    public TrapezoidProfile.Constraints getThetaConstraints() {
        return new TrapezoidProfile.Constraints(
            SwerveDriveConfig.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND.getValue(),
            SwerveDriveConfig.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED.getValue());
    }

    public Command driveAutoReset(){
        return runOnce(()->setYawCommand(getRotation2d().rotateBy(Rotation2d.fromDegrees(0)).getDegrees()));
    }  
    
    public void addSendable() {

    }

    public ChassisSpeeds getSpeeds() {//Is this Robot Relative
        return DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    public void changeAllianceRotation(){//DO THIS AT THE END OF AUTOS ONLY
        //No WORK
        setYaw(getHeading() +90);
        // switch (DriverStation.getAlliance().get()) {
        //     case Red:
        //         setYaw(getHeading() + 180);
        //         break;
        //     case Blue:
        //         setYaw(getHeading());
        //         break;
        // }
    }

    
    
    public void enableSysID() {
        sysId = new DriveSysID(swerveModules, this);
    }

    private final SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                (state) -> SignalLogger.writeString("sysid-test-state-SwerveDrive", state.toString()) // Log state with Phoenix SignalLogger class
            ),
            new SysIdRoutine.Mechanism(voltage -> {
                // Apply voltage to all drive and turn motors
                for (SwerveModule module : swerveModules) {
                  module.getDriveMotor().setVoltage(voltage);
                  module.getTurnMotor().setVoltage(voltage);
                }
              }, null, this)
            );

    public Command runSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.sysIdQuasistatic(direction);
    }

    public Command runSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.sysIdDynamic(direction);
    }
}
