package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants.Control;
import frc.utility.motor.CANMotorEx;
import frc.utility.motor.TalonEx;
import frc.utility.motor.CANMotorEx.Direction;
import frc.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.utility.template.ElevatorTemplate;
import lombok.Getter;

public class Elevator extends ElevatorTemplate{
    // 2
    //Gear Ratio: 9:1
    public static class Constants {
        private static final double GEAR_RATIO = 12.0; // motor : sprocket
        private static final double SPROCKET_TEETH = 24;
        private static final double CHAIN_PITCH_METERS = Units.inchesToMeters(0.25);


        // Derived sprocket pitch diameter
        private static final double SPROCKET_DIAMETER_METERS =
            (CHAIN_PITCH_METERS * SPROCKET_TEETH) / Math.PI;

        // Circumference for one sprocket rev
        private static final double SPROCKET_CIRCUMFERENCE_METERS =
            Math.PI * SPROCKET_DIAMETER_METERS;

        public static final double MOTOR_ROT_2_METER = SPROCKET_CIRCUMFERENCE_METERS / GEAR_RATIO;

        public static final double MIN_POSITION = 0;
        public static final double MAX_POSITION = 50.5 * Constants.MOTOR_ROT_2_METER;   //40


    }

    public enum ElevatorValue {
        START(0),
        GROUND(0),
        INTAKE_HPS(0),
        CLIMB(0),
        
        L1(0),//5
        L2(7.2),//8
        L3(22),//29

        L4(50.5),
  
        ALGAE_LOW(24.1),// 18
        ALGAE_HIGH(40),//34.5
        BARGE(50.5),
        PROCESSOR(5)//13
        ;

        private final double height;

        private ElevatorValue(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height * Constants.MOTOR_ROT_2_METER; // convert rotations to meters
        }
    }

    public double resetPos = 7 * Constants.MOTOR_ROT_2_METER;

    private static TalonEx motorRight = TalonEx.create(15)
        .withDirection(Direction.Reversed)
        .withIdleMode(ZeroPowerMode.Coast)
        .withPositionConversionFactor(1)
        .withSubsystemName("Elevator")
        .withIsEnabled(true)
        .withCurrentLimit(50);

    private static TalonEx motorLeft = TalonEx.create(14)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Coast)
        .withPositionConversionFactor(1)
        .withSubsystemName("Elevator")
        .withIsEnabled(true)
        .withCurrentLimit(50);
    
    public Elevator(boolean isEnabled) {
        super(
        new CANMotorEx[]{motorRight, motorLeft},
        new ProfiledPIDController(40, 0, 0,         
        // new TrapezoidProfile.Constraints(0.5/Constants.MOTOR_ROT_2_METER, 0.5/Constants.MOTOR_ROT_2_METER)),
        new TrapezoidProfile.Constraints(1.2, 1)), // meters per sec
        new ElevatorFeedforward(0.1, 0.18, 0.1868, 0),
        Constants.MAX_POSITION, Constants.MIN_POSITION, Constants.MOTOR_ROT_2_METER, 
        Control.TRAPEZOID_PROFILE, "Elevator", 0, isEnabled);
    }

    // public Elevator(boolean isEnabled) {
    //     super(
    //     new CANMotorEx[]{motorRight, motorLeft}, 
    //     new PIDController(0, 0, 0), //.6   
    //     new ElevatorFeedforward(0.1, 0.18, 0.1627, 0), //.1 //2.2696kv
    //     new TrapezoidProfile.Constraints(0.5, 0.5),
    //     // new ElevatorFeedforward(0.3, 0, 0.05,0), // TRAPEZOID
    //     Constants.MAX_POSITION, Constants.MIN_POSITION, Constants.MOTOR_ROT_2_METER, 
    //     Control.FEEDFORWARD, "Elevator", 0, isEnabled);
    // }

    @Override
    public void periodic() {
        super.periodic();
        // //Ensures that the encoder is always positive
        if(getPosition()<0){
            resetEncoder();
        }
    }

    public Command setTargetPositionCommand(ElevatorValue target) {
        return setTargetPositionCommand(target.getHeight());
        // return new InstantCommand(()->motorRight.setPower(1));
    }
}