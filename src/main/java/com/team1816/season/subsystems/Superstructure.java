package com.team1816.season.subsystems;

import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//Some wantedStates are still here since they're tied to aspects of the code I don't 100% understand(ClimbSide & FeederControl) -Ishwaq
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

        SHOOTER_CALIBRATE,

        SHOOTER_AUTOMATIC_HUB,
        SHOOTER_AUTOMATIC_CORNER,

        SHOOTER_DISTANCE_1,
        SHOOTER_DISTANCE_2,
        SHOOTER_DISTANCE_3,

        SNOWBLOWER_AUTOMATIC_HUB,
        SNOWBLOWER_AUTOMATIC_CORNER,

        SNOWBLOWER_DISTANCE_1,
        SNOWBLOWER_DISTANCE_2,
        SNOWBLOWER_DISTANCE_3,

        INTAKE_LIFT,
        INTAKE_DROP,

        INTAKE,
        OUTTAKE,

        GATEKEEPER_ON,
        GATEKEEPER_OFF,

        CLIMB_L1,
        CLIMB_L3,
        CLIMB_DOWN_L1,

        FEEDER_SLOW,
        FEEDER_FAST,
        FEEDER_AGITATE,

        DROP_HEIGHT
    }

    public enum ActualSuperState {
        DEFAULTING,

        SHOOTING_CALIBRATING,

        SHOOTING_AUTOMATIC_HUB,
        SHOOTING_AUTOMATIC_CORNER,

        SHOOTING_DISTANCE_1,
        SHOOTING_DISTANCE_2,
        SHOOTING_DISTANCE_3,

        SNOWBLOWING_AUTOMATIC_HUB,
        SNOWBLOWING_AUTOMATIC_CORNER,

        SNOWBLOWING_DISTANCE_1,
        SNOWBLOWING_DISTANCE_2,
        SNOWBLOWING_DISTANCE_3,

        INTAKE_LIFTING,
        INTAKE_DROPPING,

        INTAKING,
        OUTTAKING,

        GATEKEEPING_ON,
        GATEKEEPING_OFF,

        CLIMBING_L1,
        CLIMBING_L3,
        CLIMBING_DOWN_L1,

        FEEDING_SLOW,
        FEEDING_FAST,
        FEEDING_AGITATING,

        DROPPING_HEIGHT
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

    private WantedSwerveState wantedSwerveState = WantedSwerveState.MANUAL_DRIVING; //Do we need this??
    private FeederControlState feederControlState = FeederControlState.DEFAULTING; //What to do with this?

    private ClimbSide climbSide = ClimbSide.LEFT;

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
        GreenLogger.log("Wanted Superstate " + wantedSuperState);
        actualSuperState = handleStateTransitions();

        GreenLogger.log("Actual Superstate " + actualSuperState);

        applyStates();

        SmartDashboard.putString("Super state: ", wantedSuperState.toString());
    }

    private ActualSuperState handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT -> actualSuperState = ActualSuperState.DEFAULTING;

            case SHOOTER_CALIBRATE -> actualSuperState = ActualSuperState.SHOOTING_CALIBRATING;

            case SHOOTER_AUTOMATIC_HUB -> actualSuperState = ActualSuperState.SHOOTING_AUTOMATIC_HUB;
            case SHOOTER_AUTOMATIC_CORNER -> actualSuperState = ActualSuperState.SHOOTING_AUTOMATIC_CORNER;

            case SHOOTER_DISTANCE_1 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_1;
            case SHOOTER_DISTANCE_2 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_2;
            case SHOOTER_DISTANCE_3 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_3;

            case SNOWBLOWER_AUTOMATIC_HUB -> actualSuperState = ActualSuperState.SNOWBLOWING_AUTOMATIC_HUB;
            case SNOWBLOWER_AUTOMATIC_CORNER -> actualSuperState = ActualSuperState.SNOWBLOWING_AUTOMATIC_CORNER;

            case SNOWBLOWER_DISTANCE_1 -> actualSuperState = ActualSuperState.SNOWBLOWING_DISTANCE_1;
            case SNOWBLOWER_DISTANCE_2 -> actualSuperState = ActualSuperState.SNOWBLOWING_DISTANCE_2;
            case SNOWBLOWER_DISTANCE_3 -> actualSuperState = ActualSuperState.SNOWBLOWING_DISTANCE_3;

            case INTAKE_LIFT -> actualSuperState = ActualSuperState.INTAKE_LIFTING;
            case INTAKE_DROP -> actualSuperState = ActualSuperState.INTAKE_DROPPING;

            case INTAKE -> actualSuperState = ActualSuperState.INTAKING;
            case OUTTAKE -> actualSuperState = ActualSuperState.OUTTAKING;

            case GATEKEEPER_ON -> actualSuperState = ActualSuperState.GATEKEEPING_ON;
            case GATEKEEPER_OFF -> actualSuperState = ActualSuperState.GATEKEEPING_OFF;

            case CLIMB_L1 -> actualSuperState = ActualSuperState.CLIMBING_L1;
            case CLIMB_L3 -> actualSuperState = ActualSuperState.CLIMBING_L3;
            case CLIMB_DOWN_L1 -> actualSuperState = ActualSuperState.CLIMBING_DOWN_L1;

            case FEEDER_SLOW -> actualSuperState = ActualSuperState.FEEDING_SLOW;
            case FEEDER_FAST -> actualSuperState = ActualSuperState.FEEDING_FAST;
            case FEEDER_AGITATE -> actualSuperState = ActualSuperState.FEEDING_AGITATING;

            case DROP_HEIGHT -> actualSuperState = ActualSuperState.DROPPING_HEIGHT;
        }


        return actualSuperState;
    }

    private void applyStates() {
        switch (actualSuperState) {
            case DEFAULTING -> defaulting();

            case SHOOTING_CALIBRATING -> shootCalibrating();

            case SHOOTING_AUTOMATIC_HUB -> shootingAutomaticHub();
            case SHOOTING_AUTOMATIC_CORNER -> shootingAutomaticCorner();

            case SHOOTING_DISTANCE_1 -> shootingDistance1();
            case SHOOTING_DISTANCE_2 -> shootingDistance2();
            case SHOOTING_DISTANCE_3 -> shootingDistance3();

            case SNOWBLOWING_AUTOMATIC_HUB -> snowblowingAutomaticHub();
            case SNOWBLOWING_AUTOMATIC_CORNER -> snowblowingAutomaticCorner();

            case SNOWBLOWING_DISTANCE_1 -> snowblowingDistance1();
            case SNOWBLOWING_DISTANCE_2 -> snowblowingDistance2();
            case SNOWBLOWING_DISTANCE_3 -> snowblowingDistance3();

            case INTAKE_LIFTING -> intakeLifting();
            case INTAKE_DROPPING -> intakeDropping();

            case INTAKING -> intaking();
            case OUTTAKING -> outtaking();

            case GATEKEEPING_ON -> gatekeepingOn();
            case GATEKEEPING_OFF -> gatekeeperingOff();

            case CLIMBING_L1 -> climbingL1();
            case CLIMBING_L3 -> climbingL3();
            case CLIMBING_DOWN_L1 -> climbingDownL1();

            case FEEDING_SLOW -> feedingSlow();
            case FEEDING_FAST -> feedingFast();
            case FEEDING_AGITATING -> agitate();

            case DROPPING_HEIGHT -> droppingHeight();
        }
    }

    public void setWantedSuperState(WantedSuperState superState) {
        this.wantedSuperState = superState;
    }
    private void setWantedSubsystemStates(Intake.INTAKE_STATE intakeState, Feeder.FEEDER_STATE feederState,
                                          Gatekeeper.GATEKEEPER_STATE gatekeeperState, Shooter.SHOOTER_STATE shooterState,
                                          Climber.CLIMBER_STATE climbState)  {
        intake.setWantedState(intakeState);
        feeder.setWantedState(feederState);
        gatekeeper.setWantedState(gatekeeperState);
        shooter.setWantedState(shooterState);
        climber.setWantedState(climbState);
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

        swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
    }
    private void shootCalibrating() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.IDLING,
                                 Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.CALIBRATING,
                                 Climber.CLIMBER_STATE.IDLING);
    }

    private void shootingAutomaticHub() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.FAST_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.OPEN, Shooter.SHOOTER_STATE.AUTOMATIC,
            Climber.CLIMBER_STATE.IDLING); //Figure out how auto shooter is going to work
    }

    private void shootingAutomaticCorner() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.FAST_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.OPEN, Shooter.SHOOTER_STATE.AUTOMATIC,
            Climber.CLIMBER_STATE.IDLING); //Figure out how auto shooter is going to work
    }

    private void shootingDistance1() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.FAST_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.OPEN, Shooter.SHOOTER_STATE.DISTANCE_ONE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void shootingDistance2() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.FAST_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.OPEN, Shooter.SHOOTER_STATE.DISTANCE_TWO,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void shootingDistance3() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.FAST_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.OPEN, Shooter.SHOOTER_STATE.DISTANCE_THREE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void snowblowingAutomaticHub() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_IN, Feeder.FEEDER_STATE.FAST_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.OPEN, Shooter.SHOOTER_STATE.AUTOMATIC,
            Climber.CLIMBER_STATE.IDLING); //Figure out how auto shooter is going to work
    }

    private void snowblowingAutomaticCorner() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_IN, Feeder.FEEDER_STATE.FAST_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.OPEN, Shooter.SHOOTER_STATE.AUTOMATIC,
            Climber.CLIMBER_STATE.IDLING); //Figure out how auto shooter is going to work
    }

    private void snowblowingDistance1() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_IN, Feeder.FEEDER_STATE.FAST_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.OPEN, Shooter.SHOOTER_STATE.DISTANCE_ONE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void snowblowingDistance2() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_IN, Feeder.FEEDER_STATE.FAST_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.OPEN, Shooter.SHOOTER_STATE.DISTANCE_TWO,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void snowblowingDistance3() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_IN, Feeder.FEEDER_STATE.FAST_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.OPEN, Shooter.SHOOTER_STATE.DISTANCE_THREE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void intakeLifting() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE, //Would we want the shooter aiming constantly??
            Climber.CLIMBER_STATE.IDLING);
    }

    private void intakeDropping() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_DOWN, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void intaking() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_IN, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void outtaking() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_OUT, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void gatekeepingOn() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.OPEN, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void gatekeeperingOff() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void climbingL1() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_DOWN, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.L1_UP_CLIMBING);
    }

    private void climbingL3() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_DOWN, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.L3_UP_CLIMBING);
    }

    private void climbingDownL1() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_DOWN, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.L1_DOWN_CLIMBING);
    }

    private void feedingSlow() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void feedingFast() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.FAST_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void agitate() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_UP, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.IDLING);
    }

    private void droppingHeight() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_DOWN, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.IDLING);
    }


    public void setClimbSide(ClimbSide climbSide) {
        this.climbSide = climbSide;
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
            //What is supposed to be here???
    }

    public void teleopInit() {
        setWantedSuperState(WantedSuperState.DEFAULT); //We're doing this twice...
        setFeederControlState(FeederControlState.DEFAULTING); //Not sure what to do with this
        setWantedSwerveState(WantedSwerveState.MANUAL_DRIVING); //<-ditto
    }
}
