package frc.robot.subsystems.carriage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.utility.shuffleboard.ShuffleboardValue;
import lombok.Getter;

public class Carriage {

    private final ShuffleboardValue<String> positionWriter = 
        ShuffleboardValue.create("None", "CarriagePosition", "Carriage")
        .build();
    
    public enum CarriageValue{
        START(45, 230),
        INTAKE_HPS(68, 230),
        INTAKE_HPS_BLOCK(90, 211), //When Blocked by a coral at HPS
        // HPS_HOLD(134, 108),
        HOLD(INTAKE_HPS),

        INTAKE_GROUND(191 ,139),
        ALGAE_LOW(113, 214),
        ALGAE_HIGH(112, 203),
        L1(105, 234),
        L2(105, 233),
        L3(105, 230),//121//224.

        L4(113.5,243),//116,241 //!!CAN NOT BE HIGHER THAN THIS FOR PIVOT!!!
        
        BARGE(108, 133),
        BARGE_HOLD(130,130),
        PROCESSOR(105, 136),

        RESET_HIGH(115, 200)

        ;

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
        SHOOT(-700),
        // HOLD(10),
        HOLD_ALGAE(35),
        HOLD_CORAL(3),
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
        arm.setTargetPosition(CarriageValue.INTAKE_HPS.armAngle);
        pivot.setTargetPosition(CarriageValue.INTAKE_HPS.pivotAngle);
        intake.setTargetPosition(CarriageIntakeValue.STOP.intakeSpeed);
        position = CarriageValue.INTAKE_HPS;
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
            
            // isHighReset(),
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
                    // pivot.setTargetPositionCommand(100),
                    pivot.setTargetPositionCommand(targetPos.getPivotAngle()),
                    new WaitCommand(.6),
                    arm.setTargetPositionCommand(targetPos.getArmAngle())
                    // new WaitCommand(1
                    // new WaitUntilCommand(()->arm.atSetpoint()),
                    
                );
                case INTAKE_GROUND -> 
                    new SequentialCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        new WaitCommand(.5
                        ),
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
                case BARGE ->
                    new SequentialCommandGroup(
                        pivot.setTargetPositionCommand(123),
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        new WaitCommand(.5),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                    );            
                default -> 
                    new SequentialCommandGroup(
                        arm.setTargetPositionCommand(targetPos.getArmAngle()),
                        new WaitCommand(.35),
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle())
                    );
            },
            new InstantCommand(()-> position = targetPos),
            new InstantCommand(() -> positionWriter.set(position.name()))
        );
    }

    public Command setIntakeCommand(CarriageIntakeValue intakeValue){
        // DroidRageConstants.setElement(position);
        return Commands.sequence(
            coralIntake.setTargetPositionCommand(intakeValue.getIntakeSpeed())
            // DroidRageConstants.setElement(getPosition())
        );
    }

    public void incrementOuttakeCount() {
        outtakeCount++;
    }

    public boolean isCarriageValue(CarriageValue value){
        return position == value;
    }
    
    public boolean isElementIn(){
        return coralIntake.isElementIn();
    }

    // public SequentialCommandGroup isHighReset(){
    //     return new SequentialCommandGroup(
    //         new ConditionalCommand(
    //             new SequentialCommandGroup(
    //                 arm.setTargetPositionCommand(CarriageValue.RESET_HIGH.getArmAngle()),
    //                 pivot.setTargetPositionCommand(CarriageValue.RESET_HIGH.getPivotAngle()),
    //                 new WaitCommand(3)
    //             ),
    //             new InstantCommand(), 
    //             ()->position==CarriageValue.BARGE||
    //                 position==CarriageValue.L4)
    //                 // position==CarriageValue.BARGE_HOLD)
            
    //     );
    // }
}
