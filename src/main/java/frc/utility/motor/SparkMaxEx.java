package frc.utility.motor;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.DroidRageConstants;
import lombok.Getter;
import lombok.Setter;

public class SparkMaxEx extends CANMotorEx{
    public enum EncoderType {
        Relative,
        Alternate,
        Absolute
    }
    
    @Getter private final SparkMax sparkMax;
    @Setter(onMethod = @__(@Override)) private double StatorCurrentLimit; // No use; here for compatibility
    private final SparkMaxConfig config = new SparkMaxConfig();
    private Alert canAlert = new Alert("Sticky Fault", AlertType.kWarning);
    
    private SparkMaxEx(SparkMax motor) {
        this.sparkMax = motor;
    }

    public static DirectionBuilder create(int deviceID) {
        CANMotorEx motor = new SparkMaxEx(new SparkMax(deviceID, MotorType.kBrushless));
        motor.motorID = deviceID;
        return motor.new DirectionBuilder();
    }

    public static DirectionBuilder create(int deviceID, MotorType motorType) { 
        // Unlikely to be used, but it is here for compatibility with the AdvancedSixWheel project
        CANMotorEx motor = new SparkMaxEx(new SparkMax(deviceID, motorType));
        motor.motorID = deviceID;
        return motor.new DirectionBuilder();
    }

    @Override
    public void setAlert() {
        canAlert.set(sparkMax.getStickyFaults().can);
    }

    @Override
    public void setPower(double power) {
        if(isEnabledWriter.get()){
            sparkMax.set(power);
        }
        // if(DroidRageConstants.removeWriterWriter.get()){
        //     outputWriter.set(power);
        // }
    }

    @Override
    public void setVoltage(double outputVolts) {
        if(isEnabledWriter.get()){
            sparkMax.setVoltage(outputVolts);
        }
        // if(DroidRageConstants.removeWriterWriter.get()){
        //     outputWriter.set(outputVolts);
        // }
    }

    @Override
    public void setVoltage(Voltage voltage) {
        sparkMax.setVoltage(voltage);
    }
    
    @Override
    public void setDirection(Direction direction) {
        config.inverted(switch (direction) {
            case Forward -> false;
            case Reversed -> true;
        });
    }

    @Override
    public void setIdleMode(ZeroPowerMode mode) {
        config.idleMode(switch (mode) {
            case Brake -> IdleMode.kBrake;
            case Coast -> IdleMode.kCoast;
        });
    }

    // public RelativeEncoder getEncoder() {
    //     return sparkMax.getEncoder();
    // }

    // public RelativeEncoder getAlternateEncoder() {
    //     return sparkMax.getAlternateEncoder();
    // }

    // public AbsoluteEncoder getAbsoluteEncoder() {
    //     return sparkMax.getAbsoluteEncoder();
    // }

    @SuppressWarnings("unchecked")
    public <T> T getEncoder(EncoderType type) {
        switch(type) {
            case Relative: return (T) sparkMax.getEncoder();
            case Alternate: return (T) sparkMax.getAlternateEncoder();
            case Absolute: return (T) sparkMax.getAbsoluteEncoder();
            default: return null;
        }
    }

    @Override
    public double getVelocity() {
        return sparkMax.getEncoder().getVelocity();
    }

    @Override
    public double getTemp() {
        return sparkMax.getMotorTemperature();
    }

    @Override
    public double getPosition() {
        return sparkMax.getEncoder().getPosition();
    }

    public void follow(SparkMaxEx leader, boolean invert) {
        config.follow(leader.getSparkMax(), invert);
    }

    public void burnFlash() {
        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public int getDeviceID() {
        return sparkMax.getDeviceId();
    }

    @Override
    public double getSpeed(){
        return sparkMax.get();
    }

    //Casting the double to an int
    @Override
    public void setSupplyCurrentLimit(double currentLimit) {
        config.smartCurrentLimit((int) currentLimit);
        burnFlash();
    }
        
    @Override
    public double getVoltage(){
        // return motor.getAppliedOutput();//motor controller's applied output duty cycle.
        // return sparkMax.getBusVoltage();//voltage fed into the motor controller.
        return sparkMax.getOutputCurrent();//motor controller's output current in Amps.
    }

    @Override
    public void resetEncoder(int num) {
        sparkMax.getEncoder().setPosition(num);
    }

    
}