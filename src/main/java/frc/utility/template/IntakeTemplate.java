package frc.utility.template;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants.Control;
import frc.utility.motor.CANMotorEx;
import frc.utility.shuffleboard.ShuffleboardValue;

public class IntakeTemplate extends SubsystemBase{
    private final CANMotorEx[] motors;
    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;
    private final Control control;
    private final double maxSpeed;
    private final double minSpeed;
    private final ShuffleboardValue<Double> speedWriter;
    private final ShuffleboardValue<Double> targetWriter;
    private final ShuffleboardValue<Double> voltageWriter;
    private final int mainNum;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State current = new TrapezoidProfile.State(0,0); //initial
    private final TrapezoidProfile.State goal = new TrapezoidProfile.State(0,0);


    public IntakeTemplate(
        CANMotorEx[] motors,
        PIDController controller,
        SimpleMotorFeedforward feedforward,
        TrapezoidProfile.Constraints constraints,
        double maxSpeed,
        double minSpeed,
        Control control,
        String name,
        int mainNum
    ){
        this.motors=motors;
        this.controller=controller;
        this.feedforward=feedforward;
        this.control=control;
        this.maxSpeed=maxSpeed;
        this.minSpeed=minSpeed;
        this.mainNum=mainNum;

        profile = new TrapezoidProfile(constraints);

        speedWriter = ShuffleboardValue
            .create(0.0, name+"/Speed", name)
            .build();
        targetWriter = ShuffleboardValue
            .create(0.0, name+"/TargetSpeed", name)
            .build();
        voltageWriter = ShuffleboardValue
            .create(0.0, name+"/Voltage", name)
            .build();
    }

    @Override
    public void periodic() {
        switch(control){
            case PID:
                setVoltage(controller.calculate(getEncoderPosition(), controller.getSetpoint()));
                // setVoltage((controller.calculate(getEncoderPosition(), getTargetPosition())) + .37);
                //.37 is kG ^^
                break;
            case FEEDFORWARD:
                setVoltage(controller.calculate(getEncoderPosition(), controller.getSetpoint())
                +feedforward.calculate(1,1)); //To Change #
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

    public Command setTargetPositionCommand(double target){
        return new InstantCommand(()->setTargetPosition(target));
    }

    /*
     * Use this for initialization
     */
    public void setTargetPosition(double target) {
        if(target>maxSpeed||target<minSpeed) return;
        targetWriter.set(target);
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
            motor.resetEncoder(0);
        }
    }

    public double getEncoderPosition() {
        double position = motors[mainNum].getVelocity();
        speedWriter.write(position);
        return position;
    }

    public CANMotorEx getMotor() {
        return motors[mainNum];
    }

    public CANMotorEx[] getAllMotor() {
        return motors;
    }
}
