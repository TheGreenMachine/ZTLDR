package com.team1816.season.subsystems;

import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.Intake;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Superstructure extends SubsystemBase {
    private final Swerve swerve;
    private final Shooter shooter;
    private final Intake intake;
    private final Indexer indexer;
    private final Climber climber;
    protected CommandXboxController controller;

    public enum WantedSuperState {
        DEFAULT,
        CLIMB,
        IDLE
    }

    public enum ActualSuperState {
        DEFAULTING,
        CLIMBING,
        DECLIMBING,
        IDLING
    }

    public enum ClimbSide {
        // TODO: Get these poses from Choreo.
        LEFT(Pose2d.kZero),
        RIGHT(Pose2d.kZero);

        private final Pose2d climbPose;

        ClimbSide(Pose2d climbPose) {
            this.climbPose = climbPose;
        }

        public Pose2d getClimbPose() {
            return climbPose;
        }
    }

    private enum ClimbState {
        DRIVING_TO_BAR,
        DRIVING_ONTO_BAR,
        CLIMBING,
        IDLING
    }

    private enum WantedShooterState {
        AUTOMATIC_SHOOTING,
        MANUAL_SHOOTING
    }

    private enum WantedSwerveState {
        AUTOMATIC_DRIVING,
        MANUAL_DRIVING
    }

    public enum ShooterManualDistancePreset {
        DISTANCE_ONE,
        DISTANCE_TWO,
        DISTANCE_THREE,
        DISTANCE_FOUR
    }

    public enum WantedIntakeState {
        INTAKING,
        OUTTAKING,
        IDLING
    }

    public enum WantedIndexerState {
        INDEXING,
        OUTDEXING,
        IDLING
    }

    protected WantedSuperState wantedSuperState = WantedSuperState.DEFAULT;
    protected ActualSuperState actualSuperState = ActualSuperState.DEFAULTING;

    public WantedShooterState wantedShooterState = WantedShooterState.AUTOMATIC_SHOOTING;
    public WantedSwerveState wantedSwerveState = WantedSwerveState.AUTOMATIC_DRIVING;

    public ShooterManualDistancePreset shooterManualDistancePreset = ShooterManualDistancePreset.DISTANCE_ONE;
    public WantedIntakeState wantedIntakeState = WantedIntakeState.IDLING;
    public WantedIndexerState wantedIndexerState = WantedIndexerState.IDLING;

    public ClimbSide climbSide = ClimbSide.LEFT;
    public ClimbState climbState = ClimbState.IDLING;

    public Superstructure(Swerve swerve) {
        this.swerve = swerve;
        this.shooter = Singleton.get(Shooter.class);
        this.intake = Singleton.get(Intake.class);
        this.climber = Singleton.get(Climber.class);
    }

    @Override
    public void periodic() {

        actualSuperState = handleStateTransitions();

        applyStates();
    }

    private ActualSuperState handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                if (actualSuperState == ActualSuperState.CLIMBING || actualSuperState == ActualSuperState.DECLIMBING) {
                    actualSuperState = ActualSuperState.DECLIMBING;
                }
                else {
                    actualSuperState = ActualSuperState.DEFAULTING;
                }
                break;
            case CLIMB:
                actualSuperState = ActualSuperState.CLIMBING;
                break;
            case IDLE:
                actualSuperState = ActualSuperState.IDLING;
                break;
            default:
                actualSuperState = ActualSuperState.IDLING;
                break;
        }

        return actualSuperState;
    }

    protected void applyStates() {
        switch (actualSuperState) {
            case DEFAULTING:
                defaulting();
                break;
            case CLIMBING:
                climbing();
                break;
            case DECLIMBING:
                declimbing();
                break;
            case IDLING:
                break;
            default:
                actualSuperState = ActualSuperState.DEFAULTING;
                break;
        }
    }

    public void setWantedSuperState(WantedSuperState superState) {
        this.wantedSuperState = superState;
    }

    public Command setStateCommand(WantedSuperState superState) {
        return new InstantCommand(() -> setWantedSuperState(superState));
    }

    public void setClimbSide(ClimbSide climbSide) {
        this.climbSide = climbSide;
    }

    private void climbing() {
        // TODO: Add all commented functions and states
        switch (climbState) {
            case DRIVING_TO_BAR:
                swerve.setDriveToPoseTarget();
                swerve.setWantedState((Swerve.ActualState.DRIVING_TO_POSE));
                intake.setWantedState(Intake.WantedState.RETRACTING);
                shooter.setWantedState(Indexer.WantedState.RETRACTING);
                if (swerve.isAtPose()) {
                    climbState = ClimbState.DRIVING_ONTO_BAR;
                }
                break;
            case DRIVING_ONTO_BAR:
                swerve.setWantedState(Swerve.ActualState.DRIVING_TO_POSE);
                if (swerve.isAtPose() && intake.isRetracted() && shooter.isRetracted()) {
                    climbState = ClimbState.CLIMBING;
                }
                break;
            case CLIMBING:
                // This state covers all the climbing in the superstructure, the actual climbing states should be internal in the subsystem
                climber.setWantedState(Climber.WantedState.CLIMBING);
                break;
            case IDLING:
                break;
            default:
                climbState = ClimbState.IDLING;
                break;
        }
    }

    private void defaulting() {
        // TODO: Add all commented functions and states by asking Clark what it is supposed to do, and
        //  check everything mentioned by the comments

        // Make shooter handle deploying (raising) correctly
        if (wantedShooterState == WantedShooterState.AUTOMATIC_SHOOTING) {
            shooter.setWantedState(Shooter.SHOOTER_STATE.AUTOMATIC_SHOOTING);
        }
        else if (wantedShooterState == WantedShooterState.MANUAL_SHOOTING) {
            shooter.setWantedState(Shooter.SHOOTER_STATE.MANUAL_SHOOTING);
        }

        if (wantedSwerveState == WantedSwerveState.AUTOMATIC_DRIVING) {
            swerve.setWantedState(Swerve.ActualState.AUTOMATIC_DRIVING);
        }
        else if (wantedSwerveState == WantedSwerveState.MANUAL_DRIVING) {
            swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
        }

        if (shooterManualDistancePreset == ShooterManualDistancePreset.DISTANCE_ONE) {
            shooter.setManualDistancePreset(shooter.ManualDistancePreset.DISTANCE_ONE);
        }
        else if (shooterManualDistancePreset == ShooterManualDistancePreset.DISTANCE_TWO) {
            shooter.setManualDistancePreset(shooter.ManualDistancePreset.DISTANCE_TWO);
        }
        else if (shooterManualDistancePreset == ShooterManualDistancePreset.DISTANCE_THREE) {
            shooter.setManualDistancePreset(shooter.ManualDistancePreset.DISTANCE_THREE);
        }
        else if (shooterManualDistancePreset == ShooterManualDistancePreset.DISTANCE_FOUR) {
            shooter.setManualDistancePreset(shooter.ManualDistancePreset.DISTANCE_FOUR);
        }

        if (wantedIntakeState == WantedIntakeState.INTAKING) {
            intake.setWantedState(Intake.WantedState.INTAKING);
        }
        else if (wantedIntakeState == WantedIntakeState.OUTTAKING) {
            intake.setWantedState(Intake.WantedState.OUTTAKING);
        }
        else if (wantedIntakeState == WantedIntakeState.IDLING) {
            intake.setWantedState(Intake.WantedState.IDLING);
        }

        if (wantedIndexerState == WantedIndexerState.INDEXING) {
            indexer.setWantedState(Indexer.WantedState.INDEXING);
        }
        else if (wantedIndexerState == WantedIndexerState.OUTDEXING) {
            indexer.setWantedState(Indexer.WantedState.OUTDEXING);
        }
        else if (wantedIndexerState == WantedIndexerState.IDLING) {
            indexer.setWantedState(Indexer.WantedState.IDLING);
        }

        swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
    }

    public void setShooterManualDistancePreset(ShooterManualDistancePreset shooterManualDistancePreset) {
        this.shooterManualDistancePreset = shooterManualDistancePreset;
    }

    // Add functionality for this in both manual and auto shooter modes
    public void setShootingEnabled(boolean shootingEnabled) {
        shooter.setShootingEnabled(shootingEnabled);
    }

    public void setWantedIntakeState(WantedIntakeState wantedIntakeState) {
        this.wantedIntakeState = wantedIntakeState;
    }

    public void setWantedIndexerState(WantedIndexerState wantedIndexerState) {
        this.wantedIndexerState = wantedIndexerState;
    }

    public void teleopInit() {
        wantedShooterState = WantedShooterState.AUTOMATIC_SHOOTING;
        wantedIntakeState = WantedIntakeState.INTAKING;
        wantedIndexerState = WantedIndexerState.INDEXING;
        wantedSwerveState = WantedSwerveState.MANUAL_DRIVING;
    }

    public void declimbing() {
        climber.setWantedState(Climber.WantedState.DECLIMBING);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

}
