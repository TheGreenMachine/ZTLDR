package com.team1816.season.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
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
        //OLD STATES TODO: GET RID OF
        DEFAULT,
        L1_CLIMB,
        L3_CLIMB,
        L1_DOWNCLIMB,
        L3_DOWNCLIMB,
        IDLE,
        SNOWBLOWER,
        STORAGE_INTAKE,
        STORAGE_SHOOTER,
        INDEX_AGITATE,

        SHOOTER_CALIBRATE,
        SHOOTER_AUTOMATIC_HUB,
        SHOOTER_AUTOMATIC_CORNER_1,
        SHOOTER_AUTOMATIC_CORNER_2,
        SHOOTER_AUTOMATIC_CORNER_3,
        SHOOTER_AUTOMATIC_CORNER_4,
        SHOOTER_DISTANCE_1,
        SHOOTER_DISTANCE_2,
        SHOOTER_DISTANCE_3,
        SNOWBLOWER_AUTOMATIC_HUB,
        SNOWBLOWER_AUTOMATIC_CORNER_1,
        SNOWBLOWER_AUTOMATIC_CORNER_2,
        SNOWBLOWER_AUTOMATIC_CORNER_3,
        SNOWBLOWER_AUTOMATIC_CORNER_4,
        SNOWBLOWER_DISTANCE_1,
        SNOWBLOWER_DISTANCE_2,
        SNOWBLOWER_DISTANCE_3,
        INTAKE_LIFT,
        INTAKE_DROP,
        INTAKE_OUTTAKE,
        GATEKEEPER_ON,
        GATEKEEPER_OFF,
        CLIMBER_CLIMB_L1,
        CLIMBER_CLIMB_L3,
        CLIMBER_CLIMB_DOWN_L1,
        //DEFAULT,
        FEEDER_SLOW,
        FEEDER_FAST,
        HOOD_DROP



    }

    public enum ActualSuperState {
        //OLD STATES TODO: GET RID OF
        DEFAULTING,
        L1_CLIMBING,
        L3_CLIMBING,
        L1_DOWNCLIMBING,
        L3_DOWNCLIMBING,
        IDLING,
        SNOWBLOWING,
        STORAGE_INTAKING,
        STORAGE_SHOOTING,
        INDEX_AGITATING,

        SHOOTER_CALIBRATING,
        SHOOTING_AUTOMATIC_HUB,
        SHOOTING_AUTOMATIC_CORNER_1,
        SHOOTING_AUTOMATIC_CORNER_2,
        SHOOTING_AUTOMATIC_CORNER_3,
        SHOOTING_AUTOMATIC_CORNER_4,
        SHOOTING_DISTANCE_1,
        SHOOTING_DISTANCE_2,
        SHOOTING_DISTANCE_3,
        SNOWBLOWING_AUTOMATIC_HUB,
        SNOWBLOWING_AUTOMATIC_CORNER_1,
        SNOWBLOWING_AUTOMATIC_CORNER_2,
        SNOWBLOWING_AUTOMATIC_CORNER_3,
        SNOWBLOWING_AUTOMATIC_CORNER_4,
        SNOWBLOWING_DISTANCE_1,
        SNOWBLOWING_DISTANCE_2,
        SNOWBLOWING_DISTANCE_3,
        INTAKING_LIFT,
        INTAKING_DROP,
        INTAKING_OUTTAKE,
        GATEKEEPING_ON,
        GATEKEEPING_OFF,
        CLIMBING_CLIMB_L1,
        CLIMBING_CLIMB_L3,
        CLIMBING_CLIMB_DOWN_L1,
        //DEFAULTING,
        FEEDING_SLOW,
        FEEDING_FAST,
        HOOD_DROPPING
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
        SNOWBLOWING,
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
        SLOW_FEEDING,
        FAST_FEEDING,
        REVERSING,
        IDLING
    }

    public enum FeederControlState {
        OVERRIDING,
        DEFAULTING
    }

    private WantedSuperState wantedSuperState = WantedSuperState.DEFAULT;
    private ActualSuperState actualSuperState = ActualSuperState.DEFAULTING;

    // TODO: Add calibration state, maybe as the default here
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
                climber.setWantedState(Climber.CLIMBER_STATE.L1_UP_CLIMBING);
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);
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
                climber.setWantedState(Climber.CLIMBER_STATE.L3_UP_CLIMBING);
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);
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
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);
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
        shooter.setWantedState(Shooter.SHOOTER_STATE.SNOWBLOWING);
        intake.setWantedState(Intake.INTAKE_STATE.INTAKE_IN);
        feeder.setWantedState(Feeder.FEEDER_STATE.FAST_FEEDING);
        gatekeeper.setWantedState(Gatekeeper.GATEKEEPER_STATE.OPEN);
        swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
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

    public void toggleGatekeeper(){
        if (wantedGatekeeperState == WantedGatekeeperState.OPEN){
            setWantedGatekeeperState(WantedGatekeeperState.CLOSED);
        } else {
            setWantedGatekeeperState(WantedGatekeeperState.OPEN);
        }
    }

    public void toggleIntake(){
        if (wantedIntakeState == WantedIntakeState.INTAKING){
            setWantedIntakeState(WantedIntakeState.OUTTAKING);
        } else {
            setWantedIntakeState(WantedIntakeState.INTAKING);
        }
    }

    public void toggleHood(){
        if (wantedShooterState == WantedShooterState.HOOD_DOWN){
            setWantedShooterState(WantedShooterState.AUTOMATIC);
        } else {
            setWantedShooterState(WantedShooterState.HOOD_DOWN);
        }
    }

    private void storageShooting() {
        switch (wantedShooterState) {
            case AUTOMATIC:
                shooter.setWantedState(Shooter.SHOOTER_STATE.AUTOMATIC);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                break;
            case DISTANCE_ONE:
                shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_ONE);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                break;
            case DISTANCE_TWO:
                shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_TWO);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                break;
            case DISTANCE_THREE:
                shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_THREE);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                break;
            case IDLE:
                shooter.setWantedState(Shooter.SHOOTER_STATE.IDLE);
                climber.setWantedState(Climber.CLIMBER_STATE.IDLING);
                intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
                break;
            default:
                actualSuperState = ActualSuperState.IDLING;
        }
    }

    private void agitate() {
        switch(wantedFeederState) {
            case REVERSING, SLOW_FEEDING, FAST_FEEDING, IDLING:
                feeder.setWantedState(Feeder.FEEDER_STATE.REVERSING);
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
            case SLOW_FEEDING -> feeder.setWantedState(Feeder.FEEDER_STATE.SLOW_FEEDING);
            case FAST_FEEDING -> feeder.setWantedState(Feeder.FEEDER_STATE.FAST_FEEDING);
            case REVERSING -> feeder.setWantedState(Feeder.FEEDER_STATE.REVERSING);
            case IDLING -> feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
        }

        /*
         * What is this doing???
         */
        if (feederControlState == FeederControlState.OVERRIDING) {
            feeder.setWantedState(Feeder.FEEDER_STATE.FAST_FEEDING);
        }
        else {
            if (wantedIntakeState == WantedIntakeState.INTAKING || wantedGatekeeperState == WantedGatekeeperState.OPEN) {
                feeder.setWantedState(Feeder.FEEDER_STATE.SLOW_FEEDING);
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
        setWantedShooterState(WantedShooterState.AUTOMATIC);
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
