package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.carriage.Carriage;

public class RumbleCommand extends Command {
    public enum RumbleStates {
        INTAKE,
        ElEMENT_IN,
        NOTHING,
        END_GAME,
        ELEVATOR
    }

    private RumbleStates intakeState = RumbleStates.NOTHING;
    private Timer intakeTimer = new Timer();
    private Timer elementInTimer = new Timer();
    private Elevator elevator;
    private Carriage carriage;
    private CommandXboxController driver, operator;
    private double intakeTime;

    public RumbleCommand(Elevator elevator, Carriage carriage, CommandXboxController driver, CommandXboxController operator){
        this.elevator = elevator;
        this.carriage = carriage;
        this.driver = driver;
        this.operator = operator;
        addRequirements(carriage.getCoralIntake());
        intakeTime = 0;
    }

    @Override
    public void initialize() {
        intakeTimer.start();
        elementInTimer.start();
    }

    @Override
    public void execute() {
        if (getMatchTime() < 30 && getMatchTime() > 28) {
            intakeState = RumbleStates.END_GAME;
        } else if ((carriage.isElementIn() && 
            driver.getRightTriggerAxis() > 0.5 && //IsIntaking
            intakeTimer.get() > intakeTime)) {
            intakeState = RumbleStates.ElEMENT_IN;
        } else if (driver.getRightTriggerAxis() > 0.5) {
            intakeState = RumbleStates.INTAKE;
        } else if (elevator.getEncoderPosition() > -1) {
            intakeState = RumbleStates.ELEVATOR;
        } else
            intakeState = RumbleStates.NOTHING;

        switch (intakeState) {
            case ElEMENT_IN:
                driver.getHID().setRumble(RumbleType.kBothRumble, 1);
                operator.getHID().setRumble(RumbleType.kBothRumble, 1);
                DroidRageConstants.setElement(carriage.getPosition());
                break;
            case INTAKE:
                elementInTimer.reset();
                elementInTimer.start();
                intakeTime = intakeTimer.get();
            case NOTHING:
                driver.getHID().setRumble(RumbleType.kBothRumble, 0);
                operator.getHID().setRumble(RumbleType.kBothRumble, 0);
                break;
            case END_GAME:
                driver.getHID().setRumble(RumbleType.kLeftRumble, .5);
                operator.getHID().setRumble(RumbleType.kLeftRumble, .5);
                break;
            case ELEVATOR:
                operator.getHID().setRumble(RumbleType.kRightRumble, .5);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeState = RumbleStates.NOTHING;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public double getMatchTime() {// TODO:test
        return DriverStation.getMatchTime();
    }
}