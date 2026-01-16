package com.team1816.season.subsystems;

import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.Turret;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Superstructure extends SubsystemBase {
    private final Swerve swerve;
    private final Turret turret;
    protected CommandXboxController controller;

    public enum WantedSuperState {
        TELEOP_IDLE,
        TELEOP_DRIVE,
        TURRET_TO_0,
        TURRET_TO_180,
        TURRET_ROTATE_LEFT,
        TURRET_ROTATE_RIGHT,
        TURRET_IDLE
    }

    public enum CurrentSuperState {
        TELEOP_IDLE,
        TELEOP_DRIVE,
        TURRET_TO_0,
        TURRET_TO_180,
        TURRET_ROTATE_LEFT,
        TURRET_ROTATE_RIGHT,
        TURRET_IDLE
    }

    protected WantedSuperState wantedSuperState = WantedSuperState.TELEOP_IDLE;
    protected CurrentSuperState currentSuperState = CurrentSuperState.TELEOP_IDLE;
    private CurrentSuperState previousSuperState;

    public Superstructure(Swerve swerve) {
        this.swerve = swerve;
        this.turret = Singleton.get(Turret.class);
    }

    @Override
    public void periodic() {
//        boolean l = controller.leftBumper().getAsBoolean();
//        boolean r = controller.rightBumper().getAsBoolean();
//
//        if ((l && r) || (!l && !r)){
//            setWantedSuperState(WantedSuperState.TURRET_IDLE);
//        } else if (l) {
//            setWantedSuperState(WantedSuperState.TURRET_ROTATE_LEFT);
//        } else {
//            setWantedSuperState(WantedSuperState.TURRET_ROTATE_RIGHT);
//        }

        previousSuperState = currentSuperState;

        currentSuperState = handleStateTransitions();

        applyStates();
    }

    private CurrentSuperState handleStateTransitions() {
        previousSuperState = currentSuperState;
        switch (wantedSuperState) {
            case TURRET_TO_0:
                currentSuperState = CurrentSuperState.TURRET_TO_0;
                break;
            case TURRET_TO_180:
                currentSuperState = CurrentSuperState.TURRET_TO_180;
                break;
            case TURRET_IDLE:
                currentSuperState = CurrentSuperState.TURRET_IDLE;
                break;
            case TURRET_ROTATE_LEFT:
                currentSuperState = CurrentSuperState.TURRET_ROTATE_LEFT;
                break;
            case TURRET_ROTATE_RIGHT:
                currentSuperState = CurrentSuperState.TURRET_ROTATE_RIGHT;
                break;
            case TELEOP_DRIVE:
                currentSuperState = CurrentSuperState.TELEOP_DRIVE;
                break;
            default:
                currentSuperState = CurrentSuperState.TELEOP_IDLE;
                break;
        }

        return currentSuperState;
    }

    protected void applyStates() {
        switch (currentSuperState) {
            case TURRET_TO_0:
                turret.setWantedState(Turret.TURRET_STATE.TURRET_TO_0);
                swerve.setWantedState(Swerve.SWERVE_STATE.SWERVE_DRIVE);
                break;
            case TURRET_TO_180:
                turret.setWantedState(Turret.TURRET_STATE.TURRET_TO_180);
                swerve.setWantedState(Swerve.SWERVE_STATE.SWERVE_DRIVE);
                break;
            case TURRET_ROTATE_LEFT:
                turret.setWantedState(Turret.TURRET_STATE.TURRET_ROTATE_LEFT);
                swerve.setWantedState(Swerve.SWERVE_STATE.SWERVE_DRIVE);
                break;
            case TURRET_ROTATE_RIGHT:
                turret.setWantedState(Turret.TURRET_STATE.TURRET_ROTATE_RIGHT);
                swerve.setWantedState(Swerve.SWERVE_STATE.SWERVE_DRIVE);
                break;
            case TURRET_IDLE:
                turret.setWantedState(Turret.TURRET_STATE.TURRET_IDLE);
                swerve.setWantedState(Swerve.SWERVE_STATE.SWERVE_DRIVE);
                break;
            case TELEOP_DRIVE:
                turret.setWantedState(Turret.TURRET_STATE.TURRET_IDLE);
                swerve.setWantedState(Swerve.SWERVE_STATE.SWERVE_DRIVE);
                break;
            case TELEOP_IDLE:
            default:
                turret.setWantedState(Turret.TURRET_STATE.TURRET_IDLE);
                swerve.setWantedState(Swerve.SWERVE_STATE.SWERVE_IDLE);
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
