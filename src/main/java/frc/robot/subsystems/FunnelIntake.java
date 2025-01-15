package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants.Control;
import frc.robot.utility.motor.CANMotorEx;
import frc.robot.utility.motor.CANMotorEx.Direction;
import frc.robot.utility.motor.CANMotorEx.ZeroPowerMode;
import frc.robot.utility.motor.TalonEx;
import frc.robot.utility.template.IntakeTemplate;

public class FunnelIntake extends SubsystemBase {
    public enum FunnelValue{
        STOP(0,0),
        INTAKE(0,0);

        public final double intakeSpeed, beltSpeed;
        private FunnelValue(double intakeSpeed, double beltSpeed){
            this.intakeSpeed = intakeSpeed;
            this.beltSpeed = beltSpeed;
        }
    }

    private static TalonEx intakeLMotor = TalonEx.create(69)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Coast)
        .withPositionConversionFactor(1)
        .withSubsystemName("funnel")
        .withIsEnabled(true)
        .withCurrentLimit(50);
    private static TalonEx intakeRMotor = TalonEx.create(68)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Coast)
        .withPositionConversionFactor(1)
        .withSubsystemName("funnel")
        .withIsEnabled(true)
        .withCurrentLimit(50);
    private static TalonEx beltLMotor = TalonEx.create(67)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Coast)
        .withPositionConversionFactor(1)
        .withSubsystemName("funnel")
        .withIsEnabled(true)
        .withCurrentLimit(50);
    private static TalonEx beltRMotor = TalonEx.create(66)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Coast)
        .withPositionConversionFactor(1)
        .withSubsystemName("funnel")
        .withIsEnabled(true)
        .withCurrentLimit(50);

    private IntakeTemplate intake = new IntakeTemplate(
        new CANMotorEx[]{intakeLMotor, intakeRMotor}, 
        new PIDController(0,0,0), 
        new SimpleMotorFeedforward(0, 0, 0, 0), 1,0, 
        Control.PID, "carriage", 0);
    private IntakeTemplate beltIntake = new IntakeTemplate(
        new CANMotorEx[] { beltLMotor, beltRMotor },
        new PIDController(0, 0, 0),
        new SimpleMotorFeedforward(0, 0, 0, 0), 1,0,
        Control.PID, "carriage", 0);

    public FunnelIntake(boolean isEnabled) {
        intakeRMotor.setIsEnabled(isEnabled);
        intakeLMotor.setIsEnabled(isEnabled);
        beltLMotor.setIsEnabled(isEnabled);
        beltRMotor.setIsEnabled(isEnabled);
    }

    public Command setTargetPositionCommand(FunnelValue targetPos) {
        return new SequentialCommandGroup(
            intake.setTargetPositionCommand(targetPos.intakeSpeed),
            beltIntake.setTargetPositionCommand(targetPos.beltSpeed)
        );
    }
}
