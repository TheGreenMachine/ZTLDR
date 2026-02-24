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

        SHOOTER_DISTANCE_1,
        SHOOTER_DISTANCE_2,
        SHOOTER_DISTANCE_3,

        SNOWBLOWER_AUTOMATIC_CORNER,

        INTAKE_LIFT,
        INTAKE_DROP,
        INTAKE_IDLE,

        INTAKE,
        OUTTAKE,

        GATEKEEPER_ON,
        GATEKEEPER_OFF,

        CLIMB_L1,
        CLIMB_L3,
        CLIMB_DOWN_L1,

        DROP_HEIGHT
    }

    public enum ActualSuperState {
        DEFAULTING,

        SHOOTING_CALIBRATING,

        SHOOTING_AUTOMATIC_HUB,

        SHOOTING_DISTANCE_1,
        SHOOTING_DISTANCE_2,
        SHOOTING_DISTANCE_3,

        SNOWBLOWING_AUTOMATIC_CORNER,

        INTAKE_LIFTING,
        INTAKE_DROPPING,
        INTAKE_IDLING,

        INTAKING,
        OUTTAKING,

        GATEKEEPING_ON,
        GATEKEEPING_OFF,

        CLIMBING_L1,
        CLIMBING_L3,
        CLIMBING_DOWN_L1,

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

            case SHOOTER_DISTANCE_1 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_1;
            case SHOOTER_DISTANCE_2 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_2;
            case SHOOTER_DISTANCE_3 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_3;

            case INTAKE_LIFT -> actualSuperState = ActualSuperState.INTAKE_LIFTING;
            case INTAKE_DROP -> actualSuperState = ActualSuperState.INTAKE_DROPPING;
            case INTAKE_IDLE -> actualSuperState = ActualSuperState.INTAKE_IDLING;

            case INTAKE -> actualSuperState = ActualSuperState.INTAKING;
            case OUTTAKE -> actualSuperState = ActualSuperState.OUTTAKING;

            case GATEKEEPER_ON -> actualSuperState = ActualSuperState.GATEKEEPING_ON;
            case GATEKEEPER_OFF -> actualSuperState = ActualSuperState.GATEKEEPING_OFF;

            case CLIMB_L1 -> actualSuperState = ActualSuperState.CLIMBING_L1;
            case CLIMB_L3 -> actualSuperState = ActualSuperState.CLIMBING_L3;
            case CLIMB_DOWN_L1 -> actualSuperState = ActualSuperState.CLIMBING_DOWN_L1;

            case DROP_HEIGHT -> actualSuperState = ActualSuperState.DROPPING_HEIGHT;
        }


        return actualSuperState;
    }

    private void applyStates() {
        switch (actualSuperState) {
            case DEFAULTING -> defaulting();

            case SHOOTING_CALIBRATING -> shootCalibrating();

            case SHOOTING_AUTOMATIC_HUB -> shootingAutomaticHub();

            case SHOOTING_DISTANCE_1 -> shootingDistance1();
            case SHOOTING_DISTANCE_2 -> shootingDistance2();
            case SHOOTING_DISTANCE_3 -> shootingDistance3();

            case SNOWBLOWING_AUTOMATIC_CORNER -> snowblowingAutomaticCorner();

            case INTAKE_LIFTING -> intakeLifting();
            case INTAKE_DROPPING -> intakeDropping();
            case INTAKE_IDLING -> intakeIdling();

            case INTAKING -> intaking();
            case OUTTAKING -> outtaking();

            case GATEKEEPING_ON -> gatekeepingOn();
            case GATEKEEPING_OFF -> gatekeeperingOff();

            case CLIMBING_L1 -> climbingL1();
            case CLIMBING_L3 -> climbingL3();
            case CLIMBING_DOWN_L1 -> climbingDownL1();

            case DROPPING_HEIGHT -> droppingHeight();
        }
    }

    public void setWantedSuperState(WantedSuperState superState) {
        if ((superState == WantedSuperState.INTAKE) && intake.isIntaking()) {
            this.wantedSuperState = WantedSuperState.INTAKE_IDLE;
        } else if ((superState == WantedSuperState.OUTTAKE) && intake.isOutaking()) {
            this.wantedSuperState = WantedSuperState.INTAKE_IDLE;
        } else {
            this.wantedSuperState = superState;
        }
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

    private void defaulting() {
        // don't change any states, just leave us in manual drive for now
        swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
    }

    private void shootCalibrating() {
        shooter.setWantedState(Shooter.SHOOTER_STATE.CALIBRATING);
    }

    private void shootingAutomaticHub() {
        shooter.setWantedState(Shooter.SHOOTER_STATE.AUTOMATIC);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void shootingDistance1() {
        shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_ONE);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void shootingDistance2() {
        shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_TWO);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void shootingDistance3() {
        shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_THREE);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void snowblowingAutomaticCorner() {
        shooter.setWantedState(Shooter.SHOOTER_STATE.SNOWBLOWING);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void intakeLifting() {
        intake.setWantedState(Intake.INTAKE_STATE.INTAKE_UP);
        feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void intakeDropping() {
        intake.setWantedState(Intake.INTAKE_STATE.INTAKE_DOWN);
        feeder.setWantedState(Feeder.FEEDER_STATE.FAST_FEEDING);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void intaking() {
        intake.setWantedState(Intake.INTAKE_STATE.INTAKE_IN);
        feeder.setWantedState(Feeder.FEEDER_STATE.FAST_FEEDING);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void intakeIdling() {
        intake.setWantedState(Intake.INTAKE_STATE.INTAKE_DOWN);
        feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void outtaking() {
        intake.setWantedState(Intake.INTAKE_STATE.INTAKE_OUT);
        feeder.setWantedState(Feeder.FEEDER_STATE.REVERSING);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void gatekeepingOn() {
        gatekeeper.setWantedState(Gatekeeper.GATEKEEPER_STATE.OPEN);
        feeder.setWantedState(Feeder.FEEDER_STATE.FAST_FEEDING);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void gatekeeperingOff() {
        gatekeeper.setWantedState(Gatekeeper.GATEKEEPER_STATE.CLOSED);

        if (intake.isIntaking()) {
            feeder.setWantedState(Feeder.FEEDER_STATE.FAST_FEEDING);
        } else {
            feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
        }

        actualSuperState = ActualSuperState.DEFAULTING;
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

    private void droppingHeight() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_DOWN, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.IDLING);
    }

    public void setClimbSide(ClimbSide climbSide) {
        this.climbSide = climbSide;
    }

    public void autonomousInit() {
            //What is supposed to be here???
    }

    public void teleopInit() {
        swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
        intake.setWantedState(Intake.INTAKE_STATE.INTAKE_IN);
        feeder.setWantedState(Feeder.FEEDER_STATE.SLOW_FEEDING);
        shooter.setWantedState(Shooter.SHOOTER_STATE.AUTOMATIC);
    }
}
