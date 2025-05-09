package frc.utility.template;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DroidRageConstants.Control;
import frc.utility.encoder.EncoderEx;
import frc.utility.motor.CANMotorEx;
import frc.utility.motor.SparkMaxEx;

public class ArmAbsoluteTemplate extends ArmTemplate {
    protected EncoderEx encoder;
    public ArmAbsoluteTemplate(
        SparkMaxEx[] motors,
        PIDController controller,
        ArmFeedforward feedforward,
        TrapezoidProfile.Constraints constraints,
        double maxPosition,
        double minPosition,
        double offset,
        Control control,
        String tabName,
        String subsystemName,
        int mainNum,
        EncoderEx encoder
    ){
        super(motors, controller, feedforward, constraints,
        maxPosition, minPosition, offset, control, tabName,
        subsystemName, mainNum);
        this.encoder=encoder;

    }

    @Override
    public void periodic() {
        encoder.periodic();
        switch(control){
            case PID:
                setVoltage(controller.calculate(getEncoderPosition(), targetRadianWriter.get()));
                // setVoltage((controller.calculate(getEncoderPosition(), getTargetPosition())) + .37);
                //.37 is kG ^^
                break;
            case FEEDFORWARD:
                setVoltage(controller.calculate(getEncoderPosition(), targetRadianWriter.get())
                +feedforward.calculate(getEncoderPosition(),.7)); 
                // + feedforward.calculate(getTargetPosition(), .5)); 
                //ks * Math.signum(velocity) + kg * Math.cos(pos) + kv * velocity + ka * acceleration; ^^
                break;
            case TRAPEZOID_PROFILE:
                current = profile.calculate(0.02, current, goal);

                setVoltage(controller.calculate(getEncoderPosition(), current.position)
                        + feedforward.calculate(current.position, current.velocity));
                break;
        };   
    }

    @Override
    protected void setVoltage(double voltage) {
        // if (!encoder.isConnectedWriter.get()) return;
        voltageWriter.set(voltage);
        for (CANMotorEx motor: motors) {
            motor.setVoltage(voltage);
        }
    }
    
    @Override
    public double getEncoderPosition() {
        double radian = (encoder.getRadian() + offset) % (Math.PI*2);
        // double radian = encoder.getPosition();
        positionRadianWriter.write(radian);
        positionDegreeWriter.write(Math.toDegrees(radian));
        return radian;
    }
    
}
