package frc.utility.template;

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
import frc.utility.motor.MotorBase;

public class ElevatorTemplate extends SubsystemBase implements Dashboard {
    private final MotorBase[] motors;
    private PIDController controller;
    private ProfiledPIDController profiledController;
    private final ElevatorFeedforward feedforward;
    private DigitalInput limitSwitch;
    private final Control control;
    private final double maxPosition;
    private final double minPosition;
    private final double conversionFactor;
    private final int mainNum;
    private double calculatedVoltage = 0;
    private double calculatedPID = 0;
    private double calculatedFF = 0;

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
        MotorBase[] motors,
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

        // profile = new TrapezoidProfile(constraints);

        for (MotorBase motor: motors) {
            motor.withIsEnabled(isEnabled);
        }

        DashboardUtils.register(this);
        
        // controller.setTolerance(.3);
    }

    /**
     * @param motors - The Motors to Control
     * @param controller - PID Controller
     * @param feedforward - Feedforward
     * @param limitSwitch - Limit Switch REV TOUCH SENSOR
     * @param constraints
     * @param maxPosition 
     * @param minPosition
     * @param control - PID or FEEDFORWARD
     * @param name - Name of Subsystem
     * @param mainNum - Motor to use for Encoder
     */
    public ElevatorTemplate(
        MotorBase[] motors,
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

        for (MotorBase motor: motors) {
            motor.withIsEnabled(isEnabled);
        }

        // profile = new TrapezoidProfile(constraints);
        // controller.setTolerance(.3);

        DashboardUtils.register(this);
    }

    
    /**
     * Constructs an Elevator instance that uses a
     * ProfiledPIDController and feedforward for
     * control.
     * 
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
        MotorBase[] motors,
        ProfiledPIDController profiledController,
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
        this.profiledController=profiledController;
        this.feedforward=feedforward;
        this.control=control;
        this.maxPosition=maxPosition;
        this.minPosition=minPosition;
        this.conversionFactor=conversionFactor;
        this.mainNum=mainNum;

        for (MotorBase motor: motors) {
            motor.withIsEnabled(isEnabled);
        }

        // profile = new TrapezoidProfile(constraints);
        // controller.setTolerance(.3);
        // this.profiledController.setTolerance(0.001);

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
            case TRAPEZOID_PROFILE, SYS_ID:
                builder.addDoubleProperty("Goal Position", this::getGoalPosition, null);
                builder.addDoubleProperty("Current Position", this::getPosition, null);
                builder.addDoubleProperty("Position Setpoint", this::getPositionSetpoint, null);
                builder.addDoubleProperty("Velocity Setpoint", this::getVelocitySetpoint, null);
                builder.addDoubleProperty("Current Velocity", this::getVelocity, null);    
                builder.addDoubleProperty("Applied Voltage", this::getVoltage, null);            
                builder.addDoubleProperty("Calculated Voltage", () -> calculatedVoltage, null);
                builder.addDoubleProperty("Calculated FF", () -> calculatedFF, null);
                builder.addDoubleProperty("Calculated PID", () -> calculatedPID, null);
                builder.addDoubleProperty("Position Error", profiledController::getPositionError, null);
                break;

            default:
                builder.addDoubleProperty("Target Position", controller::getSetpoint, null);
                builder.addDoubleProperty("Current Position", motors[mainNum]::getPosition, null);
                builder.addDoubleProperty("Applied Voltage", () -> calculatedVoltage, null);
                break;
        }        
    }

    @Override
    public void periodic() {
        switch(control){
            case PID:
                setVoltage(controller.calculate(getPosition(), controller.getSetpoint()));
                // setVoltage((controller.calculate(getEncoderPosition(), getTargetPosition())) + .37);
                //.37 is kG ^^
                break;
            case FEEDFORWARD:
                setVoltage(controller.calculate(getPosition(), controller.getSetpoint())
                +feedforward.calculate(controller.getSetpoint())); //To Change #
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
                calculatedPID = profiledController.calculate(getPosition());

                calculatedFF = feedforward.calculate(profiledController.getSetpoint().velocity);

                setVoltage(calculatedFF + calculatedPID);

                break;
            case SYS_ID: break;
        }
        
        if(limitSwitch != null) {
            if(limitSwitch.get()) {
                setVoltage(0);
            }
        }

        // //Ensures that the encoder is always positive
        if (getPosition()<0) resetEncoder();
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
                if(target>maxPosition||target<minPosition) {
                    return;
                } else {
                    controller.setSetpoint(target);
                }
                break;
            case TRAPEZOID_PROFILE:
                if(target>maxPosition||target<minPosition) {
                    return;
                } else {
                    profiledController.setGoal(target);
                }
                break;
            case SYS_ID: break;
        }
    }
    
    public double getGoalPosition(){
        switch (control) {
            case TRAPEZOID_PROFILE: return profiledController.getGoal().position;
            default: return controller.getSetpoint();
        }
    }

    public double getVelocitySetpoint() {
        return profiledController.getSetpoint().velocity;
    }

    public double getPositionSetpoint() {
        return profiledController.getSetpoint().position;
    }
    
    protected void setVoltage(double voltage) {
        // calculatedVoltage = voltage;
        for (MotorBase motor: motors) {
            motor.setVoltage(voltage);
        }
    }

    protected void setVoltage(Voltage voltage) {
        // appliedVoltage = voltage.in(Volts);
        for (MotorBase motor: motors) {
            motor.setVoltage(voltage);
        }
    }
    
    public void resetEncoder() {
        for (MotorBase motor: motors) {
            motor.resetEncoder(0);
        }
    }

    public double getPosition() {
        return motors[mainNum].getPosition() * conversionFactor;
    }

    public double getVelocity() {
        return motors[mainNum].getVelocity() * conversionFactor;
    }

    public double getVoltage() {
        return motors[mainNum].getVoltage();
    }

    public MotorBase getMotor() {
        return motors[mainNum];
    }

    public MotorBase[] getAllMotor() {
        return motors;
    }

    public boolean atSetpoint(){
        switch (control) {
            case TRAPEZOID_PROFILE: return profiledController.atSetpoint();
            default: return controller.atSetpoint();
        }
    }

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                null, // Use default step voltage (7 volts)
                null, // Use default timeout (10 s)
                (state) -> SignalLogger.writeString("sysid-test-state-Elevator", state.toString()) // Log state with Phoenix SignalLogger class
            ), new SysIdRoutine.Mechanism(this::setVoltage, null, this));
    }

    @Override
    public void alerts() {}
}
