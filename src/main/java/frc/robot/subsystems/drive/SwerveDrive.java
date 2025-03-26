package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DroidRageConstants;
import frc.robot.SysID.DriveSysID;
import frc.robot.subsystems.drive.SwerveDriveConstants.Speed;
import frc.robot.subsystems.drive.SwerveDriveConstants.SwerveDriveConfig;
import frc.robot.subsystems.drive.SwerveModule.POD;
import frc.utility.encoder.EncoderEx.EncoderDirection;
import frc.utility.motor.CANMotorEx.Direction;
import frc.utility.motor.TalonEx;
import frc.utility.shuffleboard.ComplexWidgetBuilder;
import frc.utility.shuffleboard.ShuffleboardValue;
import lombok.Getter;

//Set Voltage instead of set Power
//Set them to 90 to 100%
public class SwerveDrive extends SubsystemBase {
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

    private volatile Speed speed = Speed.NORMAL;
    private volatile TippingState tippingState = TippingState.NO_TIP_CORRECTION;
    
    
    private final ShuffleboardValue<Double> headingWriter = 
        ShuffleboardValue.create(0.0, "Current/Gyro/Heading-Yaw (Degrees)", this.getSubsystem()).build();
    // private final ShuffleboardValue<Double> rollWriter = 
    //     ShuffleboardValue.create(0.0, "Current/Gyro/Roll (Degrees)", this.getSubsystem()).build();
    // private final ShuffleboardValue<Double> pitchWriter =   
    //     ShuffleboardValue.create(0.0, "Current/Gyro/Pitch (Degrees)", this.getSubsystem()).build();
    private final ShuffleboardValue<Boolean> isEnabledWriter = 
        ShuffleboardValue.create(true, "Is Drive Enabled", this.getSubsystem())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .build();
    protected final ShuffleboardValue<String> drivePoseWriter = ShuffleboardValue.create
        ("none", "Current/Pose", this.getSubsystem()).build();

    public SwerveDrive(Boolean isEnabled) {
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.brakeMode();
            // swerveModule.coastMode();
            // swerveModule.brakeAndCoast^Mode();
        }

        // Pigeon Wires are facing the front of the robot
        pigeon2.getConfigurator().apply(new MountPoseConfigs());   
        isEnabledWriter.set(isEnabled);
        for(int num = 0; num<4; num++){
            swerveModules[num].setDriveMotorIsEnabled(isEnabled);
            swerveModules[num].setTurnMotorIsEnabled(isEnabled);
        }    

    }

    
    @Override
    public void periodic() {
        odometry.update(
            getRotation2d(),
            getModulePositions()
        );

        drivePoseWriter.set(getPose().toString());
        headingWriter.set(getHeading());
    }

    @Override
    public void simulationPeriodic() {
        periodic();
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

    public double getHeading() {//Yaw
        return Math.IEEEremainder(pigeon2.getYaw().getValueAsDouble(), 360);
    }

    public double getPitch() {
        return Math.IEEEremainder(pigeon2.getPitch().getValueAsDouble(), 360);
    }

    public double getRoll() {
        return Math.IEEEremainder(pigeon2.getRoll().getValueAsDouble(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
        //THe negative is supposed to help work for teleop; Should FIX

    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
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
        if (!isEnabledWriter.get()) return;
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

    public TalonEx getFRTurnCanSparkMax(){
        return frontLeft.getTurnMotor();
    }

    public void changeAllianceRotation(){//DO THIS AT THE END OF AUTOS ONLY
        //No WORK
        switch (DriverStation.getAlliance().get()) {
            case Red:
                setYaw(getHeading() + 180);
                break;
            case Blue:
                setYaw(getHeading());
                break;
        }
    }

    public void enableSysID() {
        sysId = new DriveSysID(swerveModules, this);
    }

    public Command runSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.sysIdQuasistatic(direction);
    }

    public Command runSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.sysIdDynamic(direction);
    }

    
}
