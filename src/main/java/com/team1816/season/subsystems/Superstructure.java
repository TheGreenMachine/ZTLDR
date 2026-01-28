package com.team1816.season.subsystems;

import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.Intake;
import com.team1816.lib.subsystems.Turret;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Superstructure extends SubsystemBase {
    private final Swerve swerve;
    private final Turret turret;
    private final Intake intake;
    protected CommandXboxController controller;

    public enum WantedSuperState {
        DEFAULT,
        CLIMBING,
        IDLE
        }

    public enum CurrentSuperState {
        DEFAULT,
        CLIMBING,
        IDLE
    }

    protected WantedSuperState wantedSuperState = WantedSuperState.DEFAULT;
    protected CurrentSuperState currentSuperState = CurrentSuperState.DEFAULT;

    public Superstructure(Swerve swerve) {
        this.swerve = swerve;
        this.turret = Singleton.get(Turret.class);
        this.intake = Singleton.get(Intake.class);
    }

    @Override
    public void periodic() {

        currentSuperState = handleStateTransitions();

        applyStates();
    }

    private CurrentSuperState handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = CurrentSuperState.DEFAULT;
            case CLIMBING:
                currentSuperState = CurrentSuperState.CLIMBING;
            case IDLE:
                currentSuperState = CurrentSuperState.IDLE;
            default:
                currentSuperState = CurrentSuperState.IDLE;
                break;
        }

        return currentSuperState;
    }

    protected void applyStates() {
        switch (currentSuperState) {
            case DEFAULT:
                //turret.setWantedState(Turret.TURRET_STATE.TURRET_TO_0);
                //swerve.setWantedState(Swerve.SWERVE_STATE.SWERVE_DRIVE);
                break;
            case CLIMBING:
                // ?
            case IDLE:
            default:
                // ?
                break;
        }
    }

    public void setWantedSuperState(WantedSuperState superState) {
        this.wantedSuperState = superState;
    }

    public Command setStateCommand(WantedSuperState superState) {
        Command commandToReturn = new InstantCommand(() -> setWantedSuperState(superState));

        return commandToReturn;
    }

}
