package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
        public static final double MIN_POSITION = 0;
        public static final double MAX_POSITION = 55.2;   //40
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

    }

    public enum ElevatorValue {
        START(0),
        GROUND(0),
        INTAKE_HPS(0),
        CLIMB(0),
        
        L1(0),//5
        L2(7.2),//8
        L3(22),//29

        L4(54.5),
  
        ALGAE_LOW(24.1),// 18
        ALGAE_HIGH(40),//34.5
        BARGE(54.5),
        PROCESSOR(5)//13
        ;

        @Getter private final double height;

        private ElevatorValue(double height) {
            this.height = height;
        }
    }

    public double resetPos = 7;

    // GearRatio.Type type = GearRatio.Type.DISTANCE;
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
        // new PIDController(1, 0, 0), //.6
        new ProfiledPIDController(1, 0, 0,         
        new TrapezoidProfile.Constraints(0.5, 0.5)),
        new ElevatorFeedforward(0.1, 0.18, 0.1, 0), //.1 //2.2696kv
        // new ElevatorFeedforward(0.3, 0, 0.05,0), // TRAPEZOID
        Constants.MAX_POSITION, Constants.MIN_POSITION, Constants.MOTOR_ROT_2_METER, 
        Control.TRAPEZOID_PROFILE, "Elevator", 0, isEnabled);
    }

    @Override
    public void periodic() {
        super.periodic();
        // //Ensures that the encoder is always positive
        if(getEncoderPosition()<0){
            resetEncoder();
        }
    }

    public Command setTargetPositionCommand(ElevatorValue target) {
        return setTargetPositionCommand(target.getHeight());
        // return new InstantCommand(()->motorRight.setPower(1));
    }

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(7), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
            ),
            new SysIdRoutine.Mechanism((voltage) -> {
                motorLeft.setVoltage(voltage);
                motorRight.setVoltage(voltage);
            }, null, this)
        );
    }
    
    
}
