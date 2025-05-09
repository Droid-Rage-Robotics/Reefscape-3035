package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants.Control;
import frc.robot.commands.DisabledCommand;
import frc.utility.motor.CANMotorEx;
import frc.utility.motor.TalonEx;
import frc.utility.shuffleboard.ComplexWidgetBuilder;
import frc.utility.motor.CANMotorEx.Direction;
import frc.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.utility.template.ElevatorTemplate;
import lombok.Getter;

public class Elevator extends ElevatorTemplate {
    // 2
    //Gear Ratio: 9:1
    public static class Constants {
        public static final double MIN_POSITION = 0;
        public static final double MAX_POSITION = 55.2;   //40
    }

    public enum ElevatorValue {
        START(0),
        GROUND(0),
        INTAKE_HPS(0),
        CLIMB(0),
        
        L1(0),//5
        L2(7.2),//8
        L3(22),//29

        L4(54.7),//54.5
  
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
    
    private static TalonEx[] motors = {motorRight, motorLeft};
    
    public Elevator(boolean isEnabled) {
        super(
        new CANMotorEx[]{motorRight, motorLeft}, 
        new PIDController(0.9, 0, 0), //.6
        new ElevatorFeedforward(0.1, 0.18, 0, 0.), //.1
        new TrapezoidProfile.Constraints(.5, 0.5),
        Constants.MAX_POSITION,
        Constants.MIN_POSITION, 
        Control.FEEDFORWARD, "Elevator", 0);
        for (TalonEx motor: motors) {
            motor.setIsEnabled(isEnabled);
        }
        ComplexWidgetBuilder.create(DisabledCommand.create(runOnce(this::resetEncoder)), "Reset Encoder", this.getName());

        // ComplexWidgetBuilder.create(resetEncoder(), "Auto Chooser", "Misc")
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
}
