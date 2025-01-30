package frc.robot.subsystems.carriage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import lombok.Getter;

public class Carriage {
    public enum CarriageValue{
        START(0, 0),
        INTAKE_HPS(0, 0),
        INTAKE_GROUND(0, 0),
        L1(0, 0),
        L2(0, 0),
        L3(0, 0),
        L4(0, 0);

        //A Plugin tat creates a method to returns these without creating our own getters
        @Getter private final double armAngle;
        @Getter private final double pivotAngle;


        private CarriageValue(double armAngle, double pivotAngle){
            this.armAngle = armAngle;
            this.pivotAngle = pivotAngle;
        }
    }

    public enum CarriageIntakeValue {
        INTAKE(0),
        OUTTAKE(0),
        STOP(0);

        private final double intakeSpeed;

        private CarriageIntakeValue(double intakeSpeed){
            this.intakeSpeed = intakeSpeed;
        }

        public double getIntakeSpeed(){
            return intakeSpeed;
        }
    }

    @Getter private final Arm arm;
    @Getter private final Pivot pivot;
    @Getter private final Intake coralIntake;
    private final DigitalInput coralLimitSwitch;

    private CarriageValue position = CarriageValue.START;
    @Getter public double outtakeCount = 0;

    public Carriage(Arm arm, Pivot pivot, Intake intake){
        this.arm = arm;
        this.pivot = pivot;
        this.coralIntake = intake;
        this.coralLimitSwitch = new DigitalInput(0);
    }

    public CarriageValue getPosition() {
        return position;
    }
    
    public Command setPositionCommand(CarriageValue targetPos) {
        position = targetPos;
        return Commands.sequence(
            switch (targetPos) {
                case START -> 
                    new SequentialCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                    );
                case INTAKE_HPS -> 
                    new SequentialCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                    );
                case INTAKE_GROUND ->
                    new SequentialCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                    );
                case L1,L2,L3,L4 -> 
                    new SequentialCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle()),
                        new InstantCommand(()->incrementOuttakeCount())
                );                   
                default -> 
                    new ParallelCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                    );
            }
        );
    }

    public Command setIntakeCommand(CarriageIntakeValue intakeValue){
        return Commands.sequence(
            coralIntake.setTargetPositionCommand(intakeValue.getIntakeSpeed())
        );
    }

    public void incrementOuttakeCount() {
        outtakeCount++;
    }

    public boolean isCoralIn(){
        return coralLimitSwitch.get();
    }
}
