package frc.robot.SysID;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utility.motor.CANMotorEx;

public class SysID {
  public enum Measurement {
    ANGLE,
    DISTANCE
  }

  // Find Logged Data using FileZilla

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutAngle angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutDistance distance = Inches.mutable(0);
  private final MutAngularVelocity angularVelocity = RadiansPerSecond.mutable(0);
  private final MutLinearVelocity linearVelocity = InchesPerSecond.mutable(0);
  // Create a new SysId routine for characterizing the shooter.
  private SysIdRoutine routine;

  /** Creates a new Shooter subsystem. */
  public SysID(CANMotorEx motor, Subsystem subsystem, Measurement unit) {
    switch(unit){
      case ANGLE:
        routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(motor::setVoltage, log -> {
        log.motor("shooter-wheel")
            .voltage(appliedVoltage.mut_replace(motor.getVoltage() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(angle.mut_replace(motor.getPosition(), Rotations))
            .angularVelocity(angularVelocity.mut_replace(motor.getVelocity(), RotationsPerSecond));
        }, subsystem)
      );
      case DISTANCE:
        routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(motor::setVoltage, log -> {
        log.motor("shooter-wheel")
            .voltage(appliedVoltage.mut_replace(motor.getVoltage() * RobotController.getBatteryVoltage(), Volts))
            .linearPosition(distance.mut_replace(motor.getPosition(), Inches))
            .linearVelocity(linearVelocity.mut_replace(motor.getVelocity(), InchesPerSecond));
        }, subsystem)
      );
    }
  }

  /**
   * Returns a command that will execute a quasistatic test in the given
   * direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
