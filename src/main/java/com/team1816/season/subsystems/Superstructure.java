package com.team1816.season.subsystems;

import com.team1816.lib.Singleton;
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
    private final Feeder feeder;
    private final Climber climber;

    private CommandXboxController controller;

    public enum WantedSuperState {
        DEFAULT,
        L1_CLIMB,
        L3_CLIMB,
        L1_DOWNCLIMB,
        L3_DOWNCLIMB,
        IDLE,
        SNOWBLOWER,
        STORAGE_INTAKE,
        STORAGE_SHOOTER,
        INDEX_AGITATE
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
        STORAGE_SHOOTING,
        INDEX_AGITATING
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

    public enum WantedClimbState {
        IDLING,
        L3_CLIMBING,
        L3_ClIMBING_DOWN,
        L1_CLIMING,
        L1_CLIMBING_DOWN
    }

    public enum WantedShooterState {
        CALIBRATING,
        CALIBRATED,
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

    public enum WantedFeederState {
        PASSIVE_FEEDING,
        ACTIVE_FEEDING,
        AGITATING,
        IDLING
    }

    public enum FeederControlState {
        OVERRIDING,
        DEFAULTING
    }

    private WantedSuperState wantedSuperState = WantedSuperState.DEFAULT;
    private ActualSuperState actualSuperState = ActualSuperState.DEFAULTING;

    private WantedShooterState wantedShooterState = WantedShooterState.IDLE;
    private WantedGatekeeperState wantedGatekeeperState = WantedGatekeeperState.CLOSED;
    private WantedClimbState wantedClimbState = WantedClimbState.IDLING;
    private WantedSwerveState wantedSwerveState = WantedSwerveState.MANUAL_DRIVING;
    private WantedIntakeState wantedIntakeState = WantedIntakeState.UP;
    private WantedFeederState wantedFeederState = WantedFeederState.IDLING;
    private FeederControlState feederControlState = FeederControlState.DEFAULTING;

    private ClimbSide climbSide = ClimbSide.LEFT;
    private WantedClimbState climbState = WantedClimbState.IDLING;

    public Superstructure(Swerve swerve) {
        this.swerve = swerve;
        this.shooter = Singleton.get(Shooter.class);
        this.gatekeeper = Singleton.get(Gatekeeper.class);
        this.intake = Singleton.get(Intake.class);
        this.feeder = Singleton.get(Feeder.class);
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
            case INDEX_AGITATE:
                actualSuperState = ActualSuperState.INDEX_AGITATING;
                break;
            case IDLE:
            default:
                actualSuperState = ActualSuperState.IDLING;
                break;
        }

        return actualSuperState;
    }

    private void applyStates() {
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
                break;
            case STORAGE_INTAKING:
                storageIntaking();
                break;
            case SNOWBLOWING:
                snowBlowing();
                break;
            case STORAGE_SHOOTING:
                storageShooting();
                break;
            case INDEX_AGITATING:
                agitate();
                break;
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
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
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
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
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
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                actualSuperState = ActualSuperState.STORAGE_INTAKING;
                break;
            case OUTTAKING:
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_OUT);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                actualSuperState = ActualSuperState.STORAGE_INTAKING;
                break;
            case UP:
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                actualSuperState = ActualSuperState.STORAGE_INTAKING;
                break;
            case DOWN:
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_DOWN);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                actualSuperState = ActualSuperState.STORAGE_INTAKING;
                break;
            case IDLING:
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                actualSuperState = ActualSuperState.STORAGE_INTAKING;
                break;
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
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
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
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
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
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                break;
            case DISTANCE_ONE:
                shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_ONE);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                break;
            case DISTANCE_TWO:
                shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_TWO);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                break;
            case DISTANCE_THREE:
                shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_THREE);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                break;
            case IDLE:
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.IDLING);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                break;
            default:
                actualSuperState = ActualSuperState.IDLING;
        }
    }

    private void agitate() {
        switch(wantedFeederState) {
            case AGITATING, PASSIVE_FEEDING, ACTIVE_FEEDING, IDLING:
                feeder.setWantedState(Feeder.FEEDER_STATE.AGITATING);
                break;
        }
    }

    private void defaulting() {
        switch (wantedShooterState) {
            case CALIBRATING -> {if (shooter.isCalibrated()) {shooter.setWantedState(Shooter.SHOOTER_STATE.CALIBRATED);} else {shooter.setWantedState(Shooter.SHOOTER_STATE.CALIBRATING);}}
            case CALIBRATED -> {GreenLogger.log("Shooter is calibrated"); shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);}
            case DISTANCE_ONE -> shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_ONE);
            case DISTANCE_TWO -> shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_TWO);
            case DISTANCE_THREE -> shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_THREE);
            case AUTOMATIC -> shooter.setWantedState(Shooter.SHOOTER_STATE.AUTOMATIC);
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

        switch (wantedFeederState) {
            case PASSIVE_FEEDING -> feeder.setWantedState(Feeder.FEEDER_STATE.PASSIVE_FEEDING);
            case ACTIVE_FEEDING -> feeder.setWantedState(Feeder.FEEDER_STATE.ACTIVE_FEEDING);
            case AGITATING -> feeder.setWantedState(Feeder.FEEDER_STATE.AGITATING);
            case IDLING -> feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
        }

        /*
         * What is this doing???
         */
        if (feederControlState == FeederControlState.OVERRIDING) {
            feeder.setWantedState(Feeder.FEEDER_STATE.ACTIVE_FEEDING);
        }
        else {
            if (wantedIntakeState == WantedIntakeState.INTAKING || wantedGatekeeperState == WantedGatekeeperState.OPEN) {
                feeder.setWantedState(Feeder.FEEDER_STATE.PASSIVE_FEEDING);
            }
            else {
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
            }
        }

        swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
    }

    public WantedShooterState getWantedShooterState() {
        return wantedShooterState;
    }

    public void setWantedShooterState(WantedShooterState wantedShooterState) {
        this.wantedShooterState = wantedShooterState;
    }

    // Add functionality for this in both manual and auto shooter modes
    public WantedGatekeeperState getWantedGatekeeperState() {
        return wantedGatekeeperState;
    }

    public void setWantedGatekeeperState(WantedGatekeeperState gatekeeperState) {
        this.wantedGatekeeperState = gatekeeperState;
    }

    public void setFeederControlState(FeederControlState feederControlState) {
        this.feederControlState = feederControlState;
    }

    public WantedIntakeState getWantedIntakeState() {
        return wantedIntakeState;
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

    public WantedFeederState getWantedFeederState() {
        return wantedFeederState;
    }

    public void setWantedFeederState(WantedFeederState wantedFeederState) {
        this.wantedFeederState = wantedFeederState;
    }

    public WantedClimbState getWantedClimbState() {
        return wantedClimbState;
    }

    public void setWantedSwerveState(WantedSwerveState wantedSwerveState) {
        this.wantedSwerveState = wantedSwerveState;
    }

    public void autonomousInit() {
        setWantedShooterState(WantedShooterState.CALIBRATING);
    }

    public void teleopInit() {
        setWantedShooterState(WantedShooterState.AUTOMATIC);
        setWantedGatekeeperState(WantedGatekeeperState.CLOSED);
        setWantedFeederState(WantedFeederState.IDLING);
        setFeederControlState(FeederControlState.DEFAULTING);
        setFeederControlState(FeederControlState.DEFAULTING);
        setWantedIntakeState(WantedIntakeState.INTAKING);
        setWantedSwerveState(WantedSwerveState.MANUAL_DRIVING);
    }
}
