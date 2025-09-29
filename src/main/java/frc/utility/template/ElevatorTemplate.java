package frc.utility.template;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DroidRageConstants.Control;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.motor.CANMotorEx;

//Works
public class ElevatorTemplate extends SubsystemBase implements Dashboard {
    private final CANMotorEx[] motors;
    private PIDController controller;
    private ProfiledPIDController profiledController;
    private final ElevatorFeedforward feedforward;
    private DigitalInput limitSwitch;
    private final Control control;
    private final double maxPosition;
    private final double minPosition;
    private final double conversionFactor;
    private final int mainNum;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State(0,0); //initial
    private TrapezoidProfile.State currentState = new TrapezoidProfile.State(0,0); //initial
    

    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0,0);
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
        double conversionFactor,
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
        this.conversionFactor=conversionFactor;
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
        double conversionFactor,
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
        this.conversionFactor=conversionFactor;
        this.mainNum=mainNum;

        for (CANMotorEx motor: motors) {
            motor.setIsEnabled(isEnabled);
        }

        profile = new TrapezoidProfile(constraints);
        // controller.setTolerance(.3);

        DashboardUtils.register(this);
    }

    /**
     * @param motors - The Motors to Control
     * @param controller - PID Controller
     * @param feedforward - Feedforward
     * @param maxPosition 
     * @param minPosition
     * @param control - PID or FEEDFORWARD
     * @param name - Name of Subsystem
     * @param mainNum - Motor to use for Encoder
     */
    public ElevatorTemplate(
        CANMotorEx[] motors,
        ProfiledPIDController profiledcController,
        ElevatorFeedforward feedforward,
        double maxPosition,
        double minPosition,
        double conversionFactor,
        Control control,
        String name,
        int mainNum,
        boolean isEnabled
    ){
        this.motors=motors;
        this.profiledController=profiledcController;
        this.feedforward=feedforward;
        this.control=control;
        this.maxPosition=maxPosition;
        this.minPosition=minPosition;
        this.conversionFactor=conversionFactor;
        this.mainNum=mainNum;

        for (CANMotorEx motor: motors) {
            motor.setIsEnabled(isEnabled);
        }

        // profile = new TrapezoidProfile(constraints);
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
        switch(control) {
            case TRAPEZOID_PROFILE:
                builder.addDoubleProperty("Target Position", ()-> profiledController.getSetpoint().position, null);
                builder.addDoubleProperty("Current Position", motors[mainNum]::getPosition, null);
                builder.addDoubleProperty("Applied Voltage", motors[mainNum]::getVoltage, null);
                break;

            default:
                builder.addDoubleProperty("Target Position", controller::getSetpoint, null);
                builder.addDoubleProperty("Current Position", motors[mainNum]::getPosition, null);
                builder.addDoubleProperty("Applied Voltage", motors[mainNum]::getVoltage, null);
                break;
        }        
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
            // case TRAPEZOID_PROFILE:
            //     current = profile.calculate(0.02, current, goal);
                
            //     setVoltage(controller.calculate(getEncoderPosition(), current.position)
            //             + feedforward.calculate(current.position, current.velocity));
            //     break;
            case TRAPEZOID_PROFILE:
                // Advance the profile by one loop timestep (0.02s = 20ms)
                // TrapezoidProfile.State next = profile.calculate(0.02, currentState, goal);

                // double ff = feedforward.calculateWithVelocities(currentSetpoint.velocity, next.velocity);

                // double pid = controller.calculate(getEncoderPosition(), next.position);

                double pid = profiledController.calculate(getEncoderPosition());

                double ff = feedforward.calculate(profiledController.getSetpoint().velocity);

                setVoltage(ff + pid);
                // currentSetpoint = next;
                break;
            case SYS_ID: break;
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
        switch (control) {
            case PID,FEEDFORWARD:
                if(target>maxPosition||target<minPosition) return;
                controller.setSetpoint(target);
                break;
            case TRAPEZOID_PROFILE:
                if(target>maxPosition||target<minPosition) {
                    return;
                } else {
                    // goal = new TrapezoidProfile.State(target,0);
                    // currentState = new TrapezoidProfile.State(getEncoderPosition(), motors[mainNum].getVelocity());

                    profiledController.reset(getEncoderPosition(), getVelocity());
                    profiledController.setGoal(target);
                }
                break;
            case SYS_ID: break;
        }
    }
    
    public double getTargetPosition(){
        return controller.getSetpoint();
    }
    
    protected void setVoltage(double voltage) {
        for (CANMotorEx motor: motors) {
            motor.setVoltage(voltage);
        }
    }

    protected void setVoltage(Voltage voltage) {
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

    public double getVelocity() {
        return motors[mainNum].getVelocity() * conversionFactor;
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
