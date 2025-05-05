package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import frc.utility.shuffleboard.ShuffleboardValue;

public class SwerveModuleEx {
    public enum POD{
        FL,
        BL,
        FR,
        BR
    }

    private final SwerveModule<TalonFX, TalonFX, CANcoder> module;
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder turnEncoder;
    private final POD podName;

    public SwerveModuleEx(SwerveModule<TalonFX, TalonFX, CANcoder> swerveModule, POD pod) {
        this.module = swerveModule;
        this.driveMotor = swerveModule.getDriveMotor();
        this.steerMotor = swerveModule.getSteerMotor();
        this.turnEncoder = swerveModule.getEncoder();
        this.podName = pod;
    }

    public Angle getDrivePosition() {
        return driveMotor.getPosition(true).getValue();
    }
    
    public Angle getSteerPosition() {
        return turnEncoder.getAbsolutePosition(true).getValue();
    }

    public LinearVelocity getVelocity(){
        return LinearVelocity.ofRelativeUnits(module.getCurrentState().speedMetersPerSecond, MetersPerSecond);
    }

    public SwerveModulePosition getPosition() {
        return module.getPosition(true);
    }

    public SwerveModuleState getState(){
        return module.getCurrentState();
    }

    public void stop(){
        driveMotor.set(0);
        steerMotor.set(0);
    }

    public Voltage getTurnVoltage(){
        return steerMotor.getMotorVoltage().getValue();
    }

    public Voltage getDriveVoltage() {
        return driveMotor.getMotorVoltage().getValue();
    }  
}
