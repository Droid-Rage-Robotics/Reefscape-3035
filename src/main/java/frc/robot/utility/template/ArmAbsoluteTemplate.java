package frc.robot.utility.template;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.DroidRageConstants.Control;
import frc.robot.utility.encoder.EncoderEx;
import frc.robot.utility.motor.CANMotorEx;

public class ArmAbsoluteTemplate extends ArmTemplate {
    protected EncoderEx encoder;
    public ArmAbsoluteTemplate(
        CANMotorEx[] motors,
        PIDController controller,
        ArmFeedforward feedforward,
        double maxPosition,
        double minPosition,
        double offset,
        Control control,
        String subsystemName,
        int mainNum,
        EncoderEx encoder
    ){
        super(motors, controller, feedforward, 
        maxPosition, minPosition, offset, control, 
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
                +feedforward.calculate(1,1)); 
                //ks * Math.signum(velocity) + kg + kv * velocity + ka * acceleration; ^^
                break;
        };   
    }

    @Override
    protected void setVoltage(double voltage) {
        // if (encoder.isConnectedWriter.get()){
            for (CANMotorEx motor: motors) {
                motor.setVoltage(-voltage);
                //IMPORTANT: This flips the voltage to work right. Might NEED to change
            }
        // }
    }
    @Override
    public double getEncoderPosition() {
        double radian = encoder.getRadian();
        positionRadianWriter.write(radian);
        positionDegreeWriter.write(Math.toDegrees(radian));
        return radian;

        // positionRadianWriter.write(encoder.getRadian());
        // positionDegreeWriter.write(encoder.getDegrees());
        // return encoder.getDegrees();
    }
    
}
