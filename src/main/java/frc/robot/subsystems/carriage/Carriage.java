package frc.robot.subsystems.carriage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.utility.shuffleboard.ShuffleboardValue;
import lombok.Getter;

public class Carriage {

    private final ShuffleboardValue<String> positionWriter = 
        ShuffleboardValue.create("None", "CarriagePosition", "Carriage").build();
    
    public enum CarriageValue{
        START(45, 230),
        INTAKE_HPS(72, 216),
        INTAKE_HPS_BLOCK(90, 205), //When Blocked by a coral at HPS
        HOLD(INTAKE_HPS),

        INTAKE_GROUND(185,135),
        ALGAE_LOW(90, 190),
        ALGAE_HIGH(ALGAE_LOW),
        L1(105, 228),//5//245
        L2(105, 230),//109//250
        L3(L2),//121//224.

        L4(112,230),
        
        BARGE(112, 220),
        PROCESSOR(105, 135);

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
        AUTO_SHOOT(-130),
        // HOLD(10),
        HOLD_ALGAE(30),
        HOLD_CORAL(1),
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

    private CarriageValue position;
    @Getter public double outtakeCount = 0;

    public Carriage(Arm arm, Pivot pivot, Intake intake){
        this.arm = arm;
        this.pivot = pivot;
        this.coralIntake = intake;
        arm.setTargetPosition(CarriageValue.START.armAngle);
        pivot.setTargetPosition(CarriageValue.START.pivotAngle);
        intake.setTargetPosition(CarriageIntakeValue.STOP.intakeSpeed);
        position = CarriageValue.START;
        positionWriter.set(position.name());
        // this.coralLimitSwitch = new DigitalInput(0);
    }

    
    public CarriageValue getPosition() {
        return position;
    }
    
    public Command setPositionCommand(CarriageValue targetPos) {
        // position = targetPos;
        // positionWriter.set(position.name());
        return Commands.sequence(
            new InstantCommand(()-> position = targetPos),
            new InstantCommand(() -> positionWriter.set(position.name())),
            switch (targetPos) {
                case START -> 
                    new SequentialCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        new WaitCommand(3),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                    );
                case HOLD -> 
                    new SequentialCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        new WaitCommand(1.5),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                    );
                    
                case INTAKE_HPS, INTAKE_HPS_BLOCK ->  new SequentialCommandGroup(
                    pivot.setTargetPositionCommand(90),
                    arm.setTargetPositionCommand(targetPos.getArmAngle()),
                    new WaitCommand(.8),
                    pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                    
                );
                case INTAKE_GROUND -> 
                    new SequentialCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        // new WaitCommand(.1),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                        // new InstantCommand(()->incrementOuttakeCount())
                );
                case L1,L2,L3,L4 -> 
                    new SequentialCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        new WaitCommand(1),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                        // new InstantCommand(()->incrementOuttakeCount())
                );                   
                default -> 
                    new SequentialCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        new WaitCommand(1),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                    );
            }
        );
    }

    public Command setIntakeCommand(CarriageIntakeValue intakeValue){
        return Commands.sequence(
            coralIntake.setTargetPositionCommand(intakeValue.getIntakeSpeed())
            // DroidRageConstants.setElement(getPosition())
        );
    }

    public void incrementOuttakeCount() {
        outtakeCount++;
    }

    public boolean isL1(){
        return position == CarriageValue.L1;
    }
    
    public boolean isElementIn(){
        return coralIntake.isElementIn();
    }
    
}
