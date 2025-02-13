package frc.utility.motor;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.DroidRageConstants;
import lombok.Getter;
import lombok.Setter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class SparkMaxEx extends CANMotorEx{
    @Getter private final SparkMax sparkMax;
    @Setter(onMethod = @__(@Override)) private double StatorCurrentLimit; // No use; here for compatibility
    private final SparkMaxConfig config = new SparkMaxConfig();
    
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
    public void setPower(double power) {
        if(isEnabledWriter.get()){
            sparkMax.set(power);
        }
        if(DroidRageConstants.removeWriterWriter.get()){
            outputWriter.set(power);
        }
    }

    @Override
    public void setVoltage(double outputVolts) {
        if(isEnabledWriter.get()){
            sparkMax.setVoltage(outputVolts);
        }
        if(DroidRageConstants.removeWriterWriter.get()){
            outputWriter.set(outputVolts);
        }
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

    public RelativeEncoder getEncoder() {
        return sparkMax.getEncoder();
    }

    public RelativeEncoder getAlternateEncoder() {
        return sparkMax.getAlternateEncoder();
    }

    @Override
    public double getVelocity() {
        return sparkMax.getEncoder().getVelocity();
    }

    @Override
    public double getPosition() {
        return sparkMax.getEncoder().getPosition();
    }

    public SparkAbsoluteEncoder getAbsoluteEncoder() {
        return sparkMax.getAbsoluteEncoder();
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