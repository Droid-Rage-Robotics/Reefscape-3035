package frc.utility.template;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants.Control;
import frc.utility.motor.CANMotorEx;
import frc.utility.shuffleboard.ShuffleboardValue;

public class ArmTemplate extends SubsystemBase {
    protected final CANMotorEx[] motors;
    protected final PIDController controller;
    protected final ArmFeedforward feedforward;
    protected final Control control;
    protected final double maxPosition;
    protected final double minPosition;
    protected final double offset;
    protected final ShuffleboardValue<Double> positionRadianWriter;
    protected final ShuffleboardValue<Double> positionDegreeWriter;
    protected final ShuffleboardValue<Double> targetRadianWriter;
    protected final ShuffleboardValue<Double> targetDegreeWriter;
    protected final ShuffleboardValue<Double> voltageWriter;
    protected final int mainNum;
    protected final TrapezoidProfile profile;
    protected TrapezoidProfile.State current = new TrapezoidProfile.State(0,0); //initial
    protected final TrapezoidProfile.State goal = new TrapezoidProfile.State(0,0);

    public ArmTemplate(
        CANMotorEx[] motors,
        PIDController controller,
        ArmFeedforward feedforward,
        TrapezoidProfile.Constraints constraints,
        double maxPosition,
        double minPosition,
        double offset,
        Control control,
        String subsystemName,
        int mainNum
    ){
        this.motors=motors;
        this.controller=controller;
        this.feedforward=feedforward;
        this.control=control;
        this.maxPosition=maxPosition;
        this.minPosition=minPosition;
        this.offset=offset;
        this.mainNum=mainNum;

        profile = new TrapezoidProfile(constraints);

        positionDegreeWriter = ShuffleboardValue
            .create(0.0, subsystemName+"/PositionDegree", subsystemName)
            .build();
        targetDegreeWriter = ShuffleboardValue
            .create(0.0, subsystemName+"/TargetDegree", subsystemName)
            .build();
        positionRadianWriter = ShuffleboardValue
            .create(0.0, subsystemName+"/PositionRadian", subsystemName)
            .build();
        targetRadianWriter = ShuffleboardValue
            .create(0.0, subsystemName+"/TargetRadian", subsystemName)
            .build();
        voltageWriter = ShuffleboardValue
                .create(0.0, subsystemName + "/Voltage", subsystemName)
                .build();
    }

    @Override
    public void periodic() {
        // targetDegreeWriter.set(Math.toDegrees(controller.getSetpoint()));
        // targetRadianWriter.set(controller.getSetpoint());
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
            case TRAPEZOID_PROFILE:
                current = profile.calculate(0.02, current, goal);

                setVoltage(controller.calculate(getEncoderPosition(), current.position)
                        + feedforward.calculate(current.position, current.velocity));
                break;
        };        
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }
    
    public Command setTargetPositionCommand(double degree){
        return new InstantCommand(()->setTargetPosition(degree));
    }

    /*
     * Use this for initialization
     */
    public void setTargetPosition(double degree) {
        if(degree>maxPosition||degree<minPosition) return;
        targetDegreeWriter.set(degree);
        targetRadianWriter.set(Math.toRadians(degree));
        controller.setSetpoint(Math.toRadians(degree));
    }

    public double getTargetPosition(){
        return controller.getSetpoint();
    }
    protected void setVoltage(double voltage) {
        voltageWriter.set(voltage);
        for (CANMotorEx motor: motors) {
            motor.setVoltage(voltage);
        }
    }
    
    public void resetEncoder() {
        for (CANMotorEx motor: motors) {
            // motor.getEncoder().setPosition(0);
            motor.resetEncoder(0);
        }
    }

    public double getEncoderPosition() {
        double radian = motors[mainNum].getPosition()+offset;
        // + Constants.OFFSET) % Constants.RADIANS_PER_ROTATION
        positionRadianWriter.write(radian);
        positionDegreeWriter.write(Math.toDegrees(radian));
        return radian;
    }

    public CANMotorEx getMotor(){
        return motors[mainNum];
    }
    
    public CANMotorEx[] getAllMotor() {
        return motors;
    }
}
