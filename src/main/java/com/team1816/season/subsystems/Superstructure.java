package com.team1816.season.subsystems;

import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.Intake;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Superstructure extends SubsystemBase {
    private final Swerve swerve;
    private final Shooter shooter;
    private final Gatekeeper gatekeeper;
    private final Intake intake;
    private final Indexer indexer;
    private final Climber climber;
    protected CommandXboxController controller;

    public enum WantedSuperState {
        DEFAULT,
        L1_CLIMB,
        L3_CLIMB,
        L1_DOWNCLIMB,
        L3_DOWNCLIMB,
        IDLE,
        SNOWBLOWER,
        STORAGE_INTAKE,
        STORAGE_SHOOTER
    }

    public enum ActualSuperState {
        DEFAULTING,
        L1_CLIMBING,
        L3_CLIMBING,
        L1_DOWNCLIMBING,
        L3_DOWNCLIMBING,
        IDLING,
        SNOWBLOWING,
        STORAGE_INTAKING,
        STORAGE_SHOOTING
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

    private enum WantedClimbState {
        IDLING,
        L3_CLIMBING,
        L3_ClIMBING_DOWN,
        L1_CLIMING,
        L1_CLIMBING_DOWN
    }

    public enum WantedShooterState {
        DISTANCE_ONE,
        DISTANCE_TWO,
        DISTANCE_THREE,
        AUTOMATIC,
        IDLE
    }

    public enum WantedGatekeeperState {
        OPEN,
        CLOSED
    }

    public enum WantedSwerveState {
        AUTOMATIC_DRIVING,
        MANUAL_DRIVING
    }

    public enum WantedIntakeState {
        INTAKING,
        OUTTAKING,
        DOWN,
        UP,
        IDLING
    }

    public enum WantedIndexerState {
        PASSIVE_FEEDING,
        ACTIVE_FEEDING,
        AGITATING,
        IDLING
    }

    public enum IndexerControlState {
        OVERRIDING,
        DEFAULTING
    }

    protected WantedSuperState wantedSuperState = WantedSuperState.DEFAULT;
    protected ActualSuperState actualSuperState = ActualSuperState.DEFAULTING;

    public WantedShooterState wantedShooterState = WantedShooterState.IDLE;
    public WantedGatekeeperState wantedGatekeeperState = WantedGatekeeperState.CLOSED;
    public WantedSwerveState wantedSwerveState = WantedSwerveState.MANUAL_DRIVING;
    public WantedIntakeState wantedIntakeState = WantedIntakeState.UP;
    public WantedIndexerState wantedIndexerState = WantedIndexerState.IDLING;
    public IndexerControlState indexerControlState = IndexerControlState.DEFAULTING;

    public ClimbSide climbSide = ClimbSide.LEFT;
    public WantedClimbState climbState = WantedClimbState.IDLING;

    public Superstructure(Swerve swerve) {
        this.swerve = swerve;
        this.shooter = Singleton.get(Shooter.class);
        this.gatekeeper = Singleton.get(Gatekeeper.class);
        this.intake = Singleton.get(Intake.class);
        this.indexer = Singleton.get(Indexer.class);
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
                if (actualSuperState == ActualSuperState.L1_CLIMBING || actualSuperState == ActualSuperState.L1_DOWNCLIMBING) {
                    actualSuperState = ActualSuperState.L1_DOWNCLIMBING;
                }
                else {
                    actualSuperState = ActualSuperState.DEFAULTING;
                }
                break;
            case L1_CLIMB:
                actualSuperState = ActualSuperState.L1_CLIMBING;
                break;
            case L3_CLIMB:
                actualSuperState = ActualSuperState.L3_CLIMBING;
                break;
            case L1_DOWNCLIMB:
                actualSuperState = ActualSuperState.L1_DOWNCLIMBING;
                break;
            case L3_DOWNCLIMB:
                actualSuperState = ActualSuperState.L3_DOWNCLIMBING;
                break;
            case SNOWBLOWER:
                actualSuperState = ActualSuperState.SNOWBLOWING;
                break;
            case STORAGE_INTAKE:
                actualSuperState = ActualSuperState.STORAGE_INTAKING;
                break;
            case STORAGE_SHOOTER:
                actualSuperState = ActualSuperState.STORAGE_SHOOTING;
                break;
            case IDLE:
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
            case L1_CLIMBING:
                l1Climbing();
                break;
            case L1_DOWNCLIMBING:
                l1Downclimbing();
                break;
            case L3_CLIMBING:
                l3Climbing();
                break;
            case L3_DOWNCLIMBING:
                l3DownClimbing();
            case STORAGE_INTAKING:
                storageIntaking();
                break;
            case SNOWBLOWING:
                snowBlowing();
                break;
            case STORAGE_SHOOTING:
                storageShooting();
            case IDLING:
            default:
                defaulting();
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

    private void l1Climbing() {
        switch (climbState) { //WILL PROBABLY WORK DIFFERENTLY, JUST A BASIS FOR NOW
            case L1_CLIMING:
                climber.setWantedState(Climber.CLIMBER_STATE.L1_CLIMBING);
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                actualSuperState = ActualSuperState.L1_CLIMBING;
                break;
            default:
                actualSuperState = ActualSuperState.IDLING;

        }
    }

    private void l3Climbing() {
        switch (climbState) { //WILL PROBABLY WORK DIFFERENTLY, JUST A BASIS FOR NOW
            case L3_CLIMBING:
                climber.setWantedState(Climber.CLIMBER_STATE.L3_CLIMBING);
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                actualSuperState = ActualSuperState.L3_CLIMBING;
                break;
            default:
                actualSuperState = ActualSuperState.IDLING;
        }
    }

    public void storageIntaking() {
        switch (wantedIntakeState) { //WILL PROBABLY WORK DIFFERENTLY, JUST A BASIS FOR NOW
            case INTAKING:
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_IN);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                actualSuperState = ActualSuperState.STORAGE_INTAKING;
                break;
            case OUTTAKING:
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_OUT);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                actualSuperState = ActualSuperState.STORAGE_INTAKING;
                break;
            case UP:
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                actualSuperState = ActualSuperState.STORAGE_INTAKING;
                break;
            case DOWN:
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_DOWN);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                actualSuperState = ActualSuperState.STORAGE_INTAKING;
                break;
            case IDLING:
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                actualSuperState = ActualSuperState.STORAGE_INTAKING;
            default:
                actualSuperState = ActualSuperState.IDLING;

        }
    }

    public void snowBlowing() {
        //WILL NEED TO ADD MULTIPLE SUBSYSTEMS
    }


    public void l1Downclimbing() {
        switch (climbState) { //WILL PROBABLY WORK DIFFERENTLY, JUST A BASIS FOR NOW
            case L1_CLIMBING_DOWN:
                climber.setWantedState(Climber.CLIMBER_STATE.L1_DOWN_CLIMBING);
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_DOWN); //WILL NEED TO CONFIDE WITH BUILD FOR ALL WANT STATES FOR THE ACTIONS
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                actualSuperState = ActualSuperState.L1_DOWNCLIMBING;
                break;
            default:
                actualSuperState = ActualSuperState.IDLING;

        }
    }

    public void l3DownClimbing() {
        switch (climbState) { //WILL PROBABLY WORK DIFFERENTLY, JUST A BASIS FOR NOW
            case L3_ClIMBING_DOWN:
                climber.setWantedState(Climber.CLIMBER_STATE.L3_DOWN_CLIMBING);
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_DOWN);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                actualSuperState = ActualSuperState.L3_DOWNCLIMBING;
                break;
            default:
                actualSuperState = ActualSuperState.IDLING;

        }
    }

    private void storageShooting() {
        switch (wantedShooterState) {
            case AUTOMATIC:
                shooter.setWantedState(Shooter.SHOOTER_STATE.AUTOMATIC);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                break;
            case DISTANCE_ONE:
                shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_ONE);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                break;
            case DISTANCE_TWO:
                shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_TWO);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                break;
            case DISTANCE_THREE:
                shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_THREE);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                break;
            case IDLE:
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
                break;
            default:
                actualSuperState = ActualSuperState.IDLING;
        }
    }

    private void defaulting() {
        switch (wantedShooterState) {
            case DISTANCE_ONE -> shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_ONE);
            case DISTANCE_TWO -> shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_TWO);
            case DISTANCE_THREE -> shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_THREE);
            case IDLE -> shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
        }

        switch (wantedGatekeeperState) {
            case OPEN -> gatekeeper.setWantedState(Gatekeeper.GATEKEEPER_STATE.OPEN);
            case CLOSED -> gatekeeper.setWantedState(Gatekeeper.GATEKEEPER_STATE.CLOSED);
        }

        switch(wantedSwerveState) {
            case AUTOMATIC_DRIVING -> swerve.setWantedState(Swerve.ActualState.AUTOMATIC_DRIVING);
            case MANUAL_DRIVING -> swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
        }

        switch (wantedIntakeState) {
            case INTAKING -> intake.setWantedState(Intake.INTAKE_STATE.INTAKE_IN);
            case OUTTAKING -> intake.setWantedState(Intake.INTAKE_STATE.INTAKE_OUT);
            case DOWN -> intake.setWantedState(Intake.INTAKE_STATE.INTAKE_DOWN);
            case UP -> intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);

        }

        switch (wantedIndexerState) {
            case PASSIVE_FEEDING -> indexer.setWantedState(Indexer.INDEXER_STATE.PASSIVE_FEEDING);
            case ACTIVE_FEEDING -> indexer.setWantedState(Indexer.INDEXER_STATE.ACTIVE_FEEDING);
            case AGITATING -> indexer.setWantedState(Indexer.INDEXER_STATE.AGITATING);
            case IDLING -> indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
        }

        /**
         * What is this doing???
         */
        if (indexerControlState == IndexerControlState.OVERRIDING) {
            indexer.setWantedState(Indexer.INDEXER_STATE.ACTIVE_FEEDING);
        }
        else {
            if (wantedIntakeState == WantedIntakeState.INTAKING || wantedGatekeeperState == WantedGatekeeperState.OPEN) {
                indexer.setWantedState(Indexer.INDEXER_STATE.PASSIVE_FEEDING);
            }
            else {
                indexer.setWantedState(Indexer.INDEXER_STATE.IDLING);
            }
        }

        swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
    }

    public void setWantedShooterState(WantedShooterState wantedShooterState) {
        this.wantedShooterState = wantedShooterState;
    }

    // Add functionality for this in both manual and auto shooter modes
    public void setWantedGatekeeperState(WantedGatekeeperState gatekeeperState) {
        this.wantedGatekeeperState = gatekeeperState;
    }

    public void setIndexerControlState(IndexerControlState indexerControlState) {
        this.indexerControlState = indexerControlState;
    }

    public void setWantedIntakeState(WantedIntakeState wantedIntakeState) {
        if(wantedIntakeState == this.wantedIntakeState) {
            wantedIntakeState = switch (wantedIntakeState) {
                case INTAKING, OUTTAKING, DOWN -> WantedIntakeState.DOWN;
                case UP, IDLING -> WantedIntakeState.UP;
            };
        }

        this.wantedIntakeState = wantedIntakeState;
    }

    public void setWantedIndexerState(WantedIndexerState wantedIndexerState) {
        this.wantedIndexerState = wantedIndexerState;
    }

    public void setWantedSwerveState(WantedSwerveState wantedSwerveState) {
        this.wantedSwerveState = wantedSwerveState;
    }

    public void teleopInit() {
        setWantedShooterState(WantedShooterState.AUTOMATIC);
        setWantedGatekeeperState(WantedGatekeeperState.CLOSED);
        setIndexerControlState(IndexerControlState.DEFAULTING);
        setWantedIntakeState(WantedIntakeState.INTAKING);
        setWantedSwerveState(WantedSwerveState.MANUAL_DRIVING);
    }
}
