package frc.robot.subsystems.carriage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import lombok.Getter;

public class Carriage {
    public enum CarriageValue{
        START(90, 190),
        INTAKE_HPS(80, 220),
        INTAKE_HPS_BLOCK(115, 165), //When Blocked by a coral at HPS

        INTAKE_GROUND(185,135),
        ALGAE_LOW(130, 239),
        ALGAE_HIGH(130, 239),
        L1(105, 228),//5//245
        L2(105, 230),//109//250
        L3(L2),//121//224

        L4(115,225);//

        /*
        @Getter is an annotation from the lombok plugin.
        It creates a method to return stuff without creating our own getters.
        */ 
        @Getter private final double armAngle;
        @Getter private final double pivotAngle;


        private CarriageValue(double armAngle, double pivotAngle){
            this.armAngle = armAngle;
            this.pivotAngle = pivotAngle;
        }
        
        private CarriageValue(CarriageValue value) {
            this.armAngle = value.armAngle;
            this.pivotAngle = value.pivotAngle;
        }
    }

    public enum CarriageIntakeValue {
        INTAKE(50),
        OUTTAKE(-130),
        OUTTAKE_L1(-50),
        HOLD(10),
        STOP(0);

        @Getter private final double intakeSpeed;

        private CarriageIntakeValue(double intakeSpeed){
            this.intakeSpeed = intakeSpeed;
        }

        // public double getIntakeSpeed(){
        //     return intakeSpeed;
        // }
    }

    @Getter private final Arm arm;
    @Getter private final Pivot pivot;
    @Getter private final Intake coralIntake;
    // private final DigitalInput coralLimitSwitch;

    private CarriageValue position = CarriageValue.START;
    @Getter public double outtakeCount = 0;

    public Carriage(Arm arm, Pivot pivot, Intake intake){
        this.arm = arm;
        this.pivot = pivot;
        this.coralIntake = intake;
        arm.setTargetPosition(CarriageValue.START.armAngle);
        pivot.setTargetPosition(CarriageValue.START.pivotAngle);
        intake.setTargetPosition(CarriageIntakeValue.STOP.intakeSpeed);

        // this.coralLimitSwitch = new DigitalInput(0);
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
                        new WaitCommand(1.5),
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
        return coralIntake.setTargetPositionCommand(intakeValue.getIntakeSpeed());
        // return Commands.sequence(coralIntake.setTargetPositionCommand(intakeValue.getIntakeSpeed()));
        // return Commands.sequence(
            // coralIntake.setTargetPositionCommand(intakeValue.getIntakeSpeed())
        // );
    }

    public void incrementOuttakeCount() {
        outtakeCount++;
    }

    public boolean isL1(){
        return position == CarriageValue.L1;
    }
    // public boolean isCoralIn(){
    //     return coralLimitSwitch.get();
    // }
}
