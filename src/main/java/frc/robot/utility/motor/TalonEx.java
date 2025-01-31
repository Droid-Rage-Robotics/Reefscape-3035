package frc.robot.utility.motor;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.DroidRageConstants;
import lombok.Getter;

public class TalonEx extends CANMotorEx {
    @Getter private final TalonFX talon;
    @Getter private CANBus canbus;
    @Getter private double velocity;
    @Getter private double position;
    @Getter private double speed;
    @Getter private int deviceID;
    @Getter private double voltage;
    private TalonFXConfiguration configuration = new TalonFXConfiguration();
    private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    
    private TalonEx(TalonFX motor) {
        this.talon = motor;
    }

    public static DirectionBuilder create(int deviceID, CANBus canbus) {
        TalonEx motor = new TalonEx(new TalonFX(deviceID, canbus));
        motor.motorID = deviceID;
        motor.canbus = canbus;
        return motor.new DirectionBuilder();
    }
    
    public static DirectionBuilder create(int deviceID) {
        TalonEx motor = new TalonEx(new TalonFX(deviceID));
        motor.motorID = deviceID;
        return motor.new DirectionBuilder();
    }
    
    public void gettersInit() {
        velocity = talon.getVelocity().getValueAsDouble()*positionConversionFactor;
        position = talon.getPosition().getValueAsDouble()*positionConversionFactor;
        speed = talon.get();
        deviceID = talon.getDeviceID();
        voltage = talon.getMotorVoltage().getValueAsDouble();
    }
   
    @Override
    public void setDirection(Direction direction) {
        motorOutputConfigs.Inverted = switch (direction) {
            case Forward -> InvertedValue.Clockwise_Positive;
            case Reversed -> InvertedValue.CounterClockwise_Positive;
        };
        talon.getConfigurator().apply(motorOutputConfigs);
        
    }

    @Override
    public void setIdleMode(ZeroPowerMode mode) {
        talon.setNeutralMode(switch (mode) {
            case Brake -> NeutralModeValue.Brake;
            case Coast -> NeutralModeValue.Coast;
        });
    }

    @Override
    public void setSupplyCurrentLimit(double currentLimit) {
        configuration.CurrentLimits.SupplyCurrentLimit = currentLimit;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talon.getConfigurator().apply(configuration);
    }

    @Override
    public void setStatorCurrentLimit(double currentLimit){
        configuration.CurrentLimits.StatorCurrentLimit = currentLimit;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        talon.getConfigurator().apply(configuration);
    }

    @Override
    public void setPower(double power) {
    if (isEnabledWriter.get()) {
        talon.set(power);
    }
    if (DroidRageConstants.removeWriterWriter.get()) {
        outputWriter.set(power);
    }
}

    @Override
    public void setVoltage(double outputVolts) {
        if(isEnabledWriter.get()){
            talon.setVoltage(outputVolts);
        }
        if(DroidRageConstants.removeWriterWriter.get()){//if(!DriverStation.isFMSAttached())
            outputWriter.set(outputVolts);
        }
    }

    @Override
    public void setVoltage(Voltage voltage) {
        talon.setVoltage(voltage.in(Volts)/RobotController.getBatteryVoltage());
    }
    
    public void setPosition(double position) {
        talon.setPosition(position);
    }

    //Already in rotations per sec so, just covert to
    // @Override
    // public double getVelocity() {
    //     return talon.getVelocity().getValueAsDouble()*positionConversionFactor;
    // }

    // @Override
    // public double getSpeed() {
    //     return talon.get();
    // }

    // @Override
    // public double getPosition() {
    //     return talon.getPosition().getValueAsDouble()*positionConversionFactor;
    // }

    // @Override
    // public int getDeviceID() {
    //     return talon.getDeviceID();
    // }
     
    // public CANBus getCANBus() {
    //     return canbus;
    // }

    // @Override
    // public double getVoltage(){
    //     return talon.getMotorVoltage().getValueAsDouble();
    // }

    @Override
    public void resetEncoder(int num) {
        talon.setPosition(num);
    }
}
