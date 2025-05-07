package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;

public class SwerveDriveConstants {
    // public enum SwerveDriveConfig {
    //     PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND(2 * (2 * Math.PI)),
    //     TRACK_WIDTH(Units.inchesToMeters(28.5)),//Units.inchesToMeters(28.5)
    //     WHEEL_BASE(Units.inchesToMeters(28.5)),//Units.inchesToMeters(28.5)

    //     MAX_ACCELERATION_UNITS_PER_SECOND(10),
    //     MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND(10),

    //     MAX_SPEED_METERS_PER_SECOND(SwerveModule.Constants.PHYSICAL_MAX_SPEED.in(MetersPerSecond) / 4),
    //     MAX_ANGULAR_SPEED_RADIANS_PER_SECOND(PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND.getValue() / 10),
    //     MAX_ACCELERATION_METERS_PER_SECOND_SQUARED(1),
    //     MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED(1), // 1 / 8 of a full rotation per second per second),

    //     // Translational PID
    //     TRANSLATIONAL_KP(7),//3
    //     TRANSLATIONAL_KI(0),
    //     TRANSLATIONAL_KD(0),

    //     // Theta PID
    //     THETA_KP(5),
    //     THETA_KI(0),
    //     THETA_KD(0),

    //     // Turn PID for Swerve Pod
    //     TURN_KP(.5),//1

    //     // Drive SVA
    //     DRIVE_KS(0.13), // this value is multiplied by veloicty in meteres per second
    //     DRIVE_KV(2.7), //this value is the voltage that iwll be constantly applied
    //     // DRIVE_KA = 0.12,

    //     // Bevel Gears to the Left <-
    //     // BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(0),
    //     // BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(0),
    //     // FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(0),
    //     // FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(0),


    //     BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-2.26),
    //     BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(0.84),
    //     FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-1.07),
    //     FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-2.63),        

    //     DEFAULT_HEADING_OFFSET(0),
    //     ;
        
    //     public double value;
    //     private SwerveDriveConfig(double value) {
    //         this.value = value;
    //     }
        
    //     public double getValue() {
    //         return value;
    //     }
    // }

    public static class SwerveConfig {
        public static final Distance TRACK_WIDTH = Inches.of(28.5);
        public static final Distance WHEEL_BASE = Inches.of(28.5);
        
        // Bevel Gears to the Left <-
        public static final Angle BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = Radians.of(-2.26);
        public static final Angle BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = Radians.of(0.84);
        public static final Angle FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = Radians.of(-1.07);
        public static final Angle FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = Radians.of(-2.63);

        public static final PIDController TURN_PID = new PIDController(0.5, 0, 0);
        public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(0.13, 2.7);
        
        public static final PIDConstants TRANSLATIONAL_PID = new PIDConstants(7, 0, 0);
        public static final PIDConstants THETA_PID = new PIDConstants(5, 0, 0);

        public static final AngularVelocity PHYSICAL_MAX_ANGULAR_SPEED = RadiansPerSecond.of(2 * (2 * Math.PI));
        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(SwerveModule.Constants.PHYSICAL_MAX_SPEED.in(MetersPerSecond) / 4);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(PHYSICAL_MAX_ANGULAR_SPEED.in(RadiansPerSecond) / 10);

        public static final double MAX_ACCELERATION_UPS = 10; // what unit, where use?
        public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(1); // where use?
        
        public static final double MAX_ANGULAR_ACCELERATION_UPS = 10; // what unit, where use?
        
        // 1 / 8 of a full rotation per second per second
        public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(1); 

        public static final double DEFAULT_HEADING_OFFSET = 0; // where use?
    }

    public enum DriveOptions { 
        IS_FIELD_ORIENTED(true),
        IS_SQUARED_INPUTS(true),
        IS_POSE_UPDATED(true)
        ;
        // private final ShuffleboardValue<Boolean> shuffleboardValue;
        private final boolean value;
        private DriveOptions(boolean value) {
            this.value = value;
        } 
        public boolean get(){
            return value;
        }
        // @Override 
        // public ShuffleboardValue<Boolean> getNum() { return shuffleboardValue; }
    }

    public enum Speed {
        TURBO(1, 1),
        NORMAL(1.6, 1.6),//3.5, 1 //1,.4
        SLOW(.2, 0.2),
        SUPER_SLOW(0.05, 0.05),
        ;
        private final double translationalValue;
        private final double angularValue;
        private Speed(double translationalSpeed, double angularSpeed) {
            this.translationalValue = translationalSpeed;
            this.angularValue = angularSpeed;
        }
        public double getTranslationalSpeed() {
            return translationalValue;
        }
        public double getAngularSpeed() {
            return angularValue;
        }
    }
}
