package frc.utility.motor;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;

public class TalonEx extends CANMotorEx {
    private final TalonFX talon;
    private final TalonFXConfiguration config;
    private final TalonFXConfigurator configurator;
    private final Alert canAlert;
    private CANBus canbus;
    
    private TalonEx(TalonFX motor) {
        this.talon = motor;
        this.config = new TalonFXConfiguration(); // Use to change configs
        this.configurator = talon.getConfigurator(); // Use to apply configs
        canAlert = new Alert("CAN Fault", AlertType.kWarning);
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

    @Override
    public void setAlert() {
        canAlert.set(talon.getStickyFault_Hardware().getValue()); // TODO: Set to correct sticky fault        
    }
   
    @Override
    public void setDirection(Direction direction) {
        config.MotorOutput.Inverted = switch (direction) {
            case Forward -> InvertedValue.Clockwise_Positive;
            case Reversed -> InvertedValue.CounterClockwise_Positive;
        };
        configurator.apply(config);
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
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        configurator.apply(config);
    }

    @Override
    public void setStatorCurrentLimit(double currentLimit){
        config.CurrentLimits.StatorCurrentLimit = currentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        configurator.apply(config);
    }

    @Override
    public void setPower(double power) {
        if (isEnabledWriter.get()) {
            talon.set(power);
        }
        // if (DroidRageConstants.removeWriterWriter.get()) {
        //     outputWriter.set(power);
        // }
        
        tempAlertLogic();
    }

    @Override
    public void setVoltage(double outputVolts) {
        if(isEnabledWriter.get()){
            talon.setVoltage(outputVolts);
        }
        // if(DroidRageConstants.removeWriterWriter.get()){//if(!DriverStation.isFMSAttached())
        //     outputWriter.set(outputVolts);
        // }
        
        tempAlertLogic();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        talon.setVoltage(voltage.in(Volts)/RobotController.getBatteryVoltage());
    }
    
    public void setPosition(double position) {
        talon.setPosition(position);
    }

    // Already in rotations per sec so, just covert to
    @Override
    public double getVelocity() {
        return talon.getVelocity().getValueAsDouble()*positionConversionFactor;
    }

    @Override
    public double getSpeed() {
        return talon.get();
    }

    @Override
    public double getPosition() {
        return talon.getPosition().getValueAsDouble()*positionConversionFactor;
    }

    @Override
    public int getDeviceID() {
        return talon.getDeviceID();
    }
     
    public CANBus getCANBus() {
        return canbus;
    }

    @Override
    public double getVoltage(){
        return talon.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getTemp(){
        return talon.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void resetEncoder(int num) {
        talon.setPosition(num);
    }

    public void testTemp(double tempToCheck, double lowerSupply, double lowerStator){
        if(getTemp() > tempToCheck) {}
        
        // return talon.getDeviceTemp().getValueAsDouble();
    }

    public static CurrentLimitsConfigs withCurrentLimits(double supply, double stator) {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(supply)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(stator)
            .withStatorCurrentLimitEnable(true);
        return limits;
    }

    public static CurrentLimitsConfigs withCurrentLimits(double supply) {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(supply)
            .withSupplyCurrentLimitEnable(true);
        return limits;
    }

    
}
