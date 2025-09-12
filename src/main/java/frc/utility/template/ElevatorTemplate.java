package frc.utility.template;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants.Control;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.motor.CANMotorEx;

//Works
public class ElevatorTemplate extends SubsystemBase implements Dashboard {
    private final CANMotorEx[] motors;
    private final PIDController controller;
    private final ElevatorFeedforward feedforward;
    private DigitalInput limitSwitch;
    private final Control control;
    private final double maxPosition;
    private final double minPosition;
    private final int mainNum;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State current = new TrapezoidProfile.State(0,0); //initial
    private final TrapezoidProfile.State goal = new TrapezoidProfile.State(0,0);
    // REV TOUCH SENSOR

    /**
     * @param motors - The Motors to Control
     * @param controller - PID Controller
     * @param feedforward - Feedforward
     * @param constraints
     * @param maxPosition 
     * @param minPosition
     * @param control - PID or FEEDFORWARD
     * @param name - Name of Subsystem
     * @param mainNum - Motor to use for Encoder
     */
    public ElevatorTemplate(
        CANMotorEx[] motors,
        PIDController controller,
        ElevatorFeedforward feedforward,
        TrapezoidProfile.Constraints constraints,
        double maxPosition,
        double minPosition,
        Control control,
        String name,
        int mainNum,
        boolean isEnabled
    ){
        this.motors=motors;
        this.controller=controller;
        this.feedforward=feedforward;
        this.control=control;
        this.maxPosition=maxPosition;
        this.minPosition=minPosition;
        this.mainNum=mainNum;

        profile = new TrapezoidProfile(constraints);

        for (CANMotorEx motor: motors) {
            motor.setIsEnabled(isEnabled);
        }

        DashboardUtils.register(this);
        
        // controller.setTolerance(.3);
    }

    /**
     * @param motors - The Motors to Control
     * @param controller - PID Controller
     * @param feedforward - Feedforward
     * @param limitSwitch - Limit Switch
     * @param constraints
     * @param maxPosition 
     * @param minPosition
     * @param control - PID or FEEDFORWARD
     * @param name - Name of Subsystem
     * @param mainNum - Motor to use for Encoder
     */
    public ElevatorTemplate(
        CANMotorEx[] motors,
        PIDController controller,
        ElevatorFeedforward feedforward,
        DigitalInput limitSwitch,
        TrapezoidProfile.Constraints constraints,
        double maxPosition,
        double minPosition,
        Control control,
        String name,
        int mainNum,
        boolean isEnabled
    ){
        this.motors=motors;
        this.controller=controller;
        this.feedforward=feedforward;
        this.limitSwitch=limitSwitch;
        this.control=control;
        this.maxPosition=maxPosition;
        this.minPosition=minPosition;
        this.mainNum=mainNum;

        for (CANMotorEx motor: motors) {
            motor.setIsEnabled(isEnabled);
        }

        profile = new TrapezoidProfile(constraints);
        // controller.setTolerance(.3);

        DashboardUtils.register(this);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Elevator", this);
        SmartDashboard.putData("Elevator/Reset Encoder", runOnce(this::resetEncoder));
    }

    @Override
    public void practiceWriters() {
        
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Position", controller::getSetpoint, null);
        builder.addDoubleProperty("Current Position", motors[mainNum]::getPosition, null);
        builder.addDoubleProperty("Applied Voltage", motors[mainNum]::getVoltage, null);
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
            // case FEEDFORWARD:
            //     setVoltage(controller.calculate(getEncoderPosition(), controller.getSetpoint())
            //     +feedforward.calculateWithVelocities(1, 1));
            //     break;
            case TRAPEZOID_PROFILE:
                current = profile.calculate(0.02, current, goal);
                
                setVoltage(controller.calculate(getEncoderPosition(), current.position)
                        + feedforward.calculate(current.position, current.velocity));
                break;
            // case TRAPEZOID_PROFILE:
            //     // Advance the profile by one loop timestep (0.02s = 20ms)
            //     TrapezoidProfile.State next = profile.calculate(0.02, current, goal);

            //     double ff = feedforward.calculateWithVelocities(current.velocity, next.velocity);

            //     double pid = controller.calculate(getEncoderPosition(), controller.getSetpoint());

            //     setVoltage(ff + pid);
            //     current = next;
            //     break;
        }       
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
        if(target>maxPosition||target<minPosition) return;
        controller.setSetpoint(target);
    }
    
    public double getTargetPosition(){
        return controller.getSetpoint();
    }
    
    protected void setVoltage(double voltage) {
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
        return motors[mainNum].getPosition();
    }

    public CANMotorEx getMotor() {
        return motors[mainNum];
    }

    public CANMotorEx[] getAllMotor() {
        return motors;
    }

    public boolean atSetPoint(){
        return controller.atSetpoint();
    }
}
