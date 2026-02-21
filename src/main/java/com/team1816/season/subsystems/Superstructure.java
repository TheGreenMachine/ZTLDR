package com.team1816.season.subsystems;

import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.drivetrain.Swerve;
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
        CLIMB_L1,
        CLIMB_L3,
        CLIMB_DOWN_L1,
        FEEDER_SLOW,
        FEEDER_FAST,
        FEEDER_AGITATE,
        HOOD_DROP



    }

    public enum ActualSuperState {

        DEFAULTING,
        SHOOTING_CALIBRATING,
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
        INTAKE_LIFTING,
        INTAKE_DROPPING,
        INTAKING_OUTTAKING,
        GATEKEEPING_ON,
        GATEKEEPING_OFF,
        CLIMBING_L1,
        CLIMBING_L3,
        CLIMBING_DOWN_L1,
        FEEDING_SLOW,
        FEEDING_FAST,
        FEEDING_AGITATE,
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

    public enum WantedSwerveState {
        AUTOMATIC_DRIVING,
        MANUAL_DRIVING
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
    private Shooter.SHOOTER_STATE wantedShooterState = Shooter.SHOOTER_STATE.IDLE;
    private Gatekeeper.GATEKEEPER_STATE wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.CLOSED;
    private Climber.CLIMBER_STATE wantedClimbState = Climber.CLIMBER_STATE.IDLING;
    private WantedSwerveState wantedSwerveState = WantedSwerveState.MANUAL_DRIVING; //Do we need this??
    private Intake.INTAKE_STATE wantedIntakeState = Intake.INTAKE_STATE.INTAKE_UP;
    private Feeder.FEEDER_STATE wantedFeederState = Feeder.FEEDER_STATE.IDLING;
    private FeederControlState feederControlState = FeederControlState.DEFAULTING; //What to do with this?
    private Climber.CLIMBER_STATE climbState = Climber.CLIMBER_STATE.IDLING;

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
    }

    private ActualSuperState handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT -> actualSuperState = ActualSuperState.DEFAULTING;
            case SHOOTER_CALIBRATE -> actualSuperState = ActualSuperState.SHOOTING_CALIBRATING;
            case SHOOTER_AUTOMATIC_HUB -> actualSuperState = ActualSuperState.SHOOTING_AUTOMATIC_HUB;
            case SHOOTER_AUTOMATIC_CORNER_1 -> actualSuperState = ActualSuperState.SHOOTING_AUTOMATIC_CORNER_1;
            case SHOOTER_AUTOMATIC_CORNER_2 -> actualSuperState = ActualSuperState.SHOOTING_AUTOMATIC_CORNER_2;
            case SHOOTER_AUTOMATIC_CORNER_3 -> actualSuperState = ActualSuperState.SHOOTING_AUTOMATIC_CORNER_3;
            case SHOOTER_AUTOMATIC_CORNER_4 -> actualSuperState = ActualSuperState.SHOOTING_AUTOMATIC_CORNER_4;
            case SHOOTER_DISTANCE_1 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_1;
            case SHOOTER_DISTANCE_2 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_2;
            case SHOOTER_DISTANCE_3 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_3;
            case SNOWBLOWER_AUTOMATIC_HUB -> actualSuperState = ActualSuperState.SNOWBLOWING_AUTOMATIC_HUB;
            case SNOWBLOWER_AUTOMATIC_CORNER_1 -> actualSuperState = ActualSuperState.SNOWBLOWING_AUTOMATIC_CORNER_1;
            case SNOWBLOWER_AUTOMATIC_CORNER_2 -> actualSuperState = ActualSuperState.SNOWBLOWING_AUTOMATIC_CORNER_2;
            case SNOWBLOWER_AUTOMATIC_CORNER_3 -> actualSuperState = ActualSuperState.SNOWBLOWING_AUTOMATIC_CORNER_3;
            case SNOWBLOWER_AUTOMATIC_CORNER_4 -> actualSuperState = ActualSuperState.SNOWBLOWING_AUTOMATIC_CORNER_4;
            case SNOWBLOWER_DISTANCE_1 -> actualSuperState = ActualSuperState.SNOWBLOWING_DISTANCE_1;
            case SNOWBLOWER_DISTANCE_2 -> actualSuperState = ActualSuperState.SNOWBLOWING_DISTANCE_2;
            case SNOWBLOWER_DISTANCE_3 -> actualSuperState = ActualSuperState.SNOWBLOWING_DISTANCE_3;
            case INTAKE_LIFT -> actualSuperState = ActualSuperState.INTAKE_LIFTING;
            case INTAKE_DROP -> actualSuperState = ActualSuperState.INTAKE_DROPPING;
            case INTAKE_OUTTAKE -> actualSuperState = ActualSuperState.INTAKING_OUTTAKING;
            case GATEKEEPER_ON -> actualSuperState = ActualSuperState.GATEKEEPING_ON;
            case GATEKEEPER_OFF -> actualSuperState = ActualSuperState.GATEKEEPING_OFF;
            case CLIMB_L1 -> actualSuperState = ActualSuperState.CLIMBING_L1;
            case CLIMB_L3 -> actualSuperState = ActualSuperState.CLIMBING_L3;
            case CLIMB_DOWN_L1 -> actualSuperState = ActualSuperState.CLIMBING_DOWN_L1;
            case FEEDER_SLOW -> actualSuperState = ActualSuperState.FEEDING_SLOW;
            case FEEDER_FAST -> actualSuperState = ActualSuperState.FEEDING_FAST;
            case HOOD_DROP -> actualSuperState = ActualSuperState.HOOD_DROPPING;
        }


        return actualSuperState;
    }

    private void applyStates() {
        switch (actualSuperState) {
            case DEFAULTING -> defaulting();
            case SHOOTING_CALIBRATING -> shootingCalibrating();
            case SHOOTING_AUTOMATIC_HUB -> shootingAutomaticHub();
            case SHOOTING_AUTOMATIC_CORNER_1 -> shootingAutomaticCorner1();
            case SHOOTING_AUTOMATIC_CORNER_2 -> shootingAutomaticCorner2();
            case SHOOTING_AUTOMATIC_CORNER_3 -> shootingAutomaticCorner3();
            case SHOOTING_AUTOMATIC_CORNER_4 -> shootingAutomaticCorner4();
            case SHOOTING_DISTANCE_1 -> shootingDistance1();
            case SHOOTING_DISTANCE_2 -> shootingDistance2();
            case SHOOTING_DISTANCE_3 -> shootingDistance3();
            case SNOWBLOWING_AUTOMATIC_HUB -> snowblowingAutomaticHub();
            case SNOWBLOWING_AUTOMATIC_CORNER_1 -> snowblowingAutomaticCorner1();
            case SNOWBLOWING_AUTOMATIC_CORNER_2 -> snowblowingAutomaticCorner2();
            case SNOWBLOWING_AUTOMATIC_CORNER_3 -> snowblowingAutomaticCorner3();
            case SNOWBLOWING_AUTOMATIC_CORNER_4 -> snowblowingAutomaticCorner4();
            case SNOWBLOWING_DISTANCE_1 -> snowblowingDistance1();
            case SNOWBLOWING_DISTANCE_2 -> snowblowingDistance2();
            case SNOWBLOWING_DISTANCE_3 -> snowblowingDistance3();
            case INTAKE_LIFTING -> intakeLifting();
            case INTAKE_DROPPING -> intakeDropping();
            case GATEKEEPING_ON -> gatekeepingOn();
            case GATEKEEPING_OFF -> gatekeeperingOff();
            case CLIMBING_L1 -> climbingL1();
            case CLIMBING_L3 -> climbingL3();
            case CLIMBING_DOWN_L1 -> climbingDownL1();
            case FEEDING_SLOW -> feedingSlow();
            case FEEDING_FAST -> feedingFast();
            case FEEDING_AGITATE -> agitate();
            case HOOD_DROPPING -> hoodDropping();
        }
    }

    public void setWantedSuperState(WantedSuperState superState) {
        this.wantedSuperState = superState;
    }

    public Command setStateCommand(WantedSuperState superState) {
        return new InstantCommand(() -> setWantedSuperState(superState));
    }
    private void defaulting() {}
    private void shootingCalibrating() {
        wantedShooterState = Shooter.SHOOTER_STATE.CALIBRATING;
    }
    private void shootingAutomaticHub() {}
    private void shootingAutomaticCorner1() {}
    private void shootingAutomaticCorner2() {}
    private void shootingAutomaticCorner3() {}
    private void shootingAutomaticCorner4() {}
    private void shootingDistance1() {}
    private void shootingDistance2() {}
    private void shootingDistance3() {}
    private void snowblowingAutomaticHub() {}
    private void snowblowingAutomaticCorner1() {}
    private void snowblowingAutomaticCorner2() {}
    private void snowblowingAutomaticCorner3() {}
    private void snowblowingAutomaticCorner4() {}
    private void snowblowingDistance1() {}
    private void snowblowingDistance2() {}
    private void snowblowingDistance3() {}
    private void intakeLifting() {}
    private void intakeDropping() {}
    private void gatekeepingOn() {}
    private void gatekeeperingOff() {}
    private void climbingL1() {}
    private void climbingL3() {}
    private void climbingDownL1() {}
    private void feedingSlow() {}
    private void feedingFast() {}

    private void agitate() {
        switch(wantedFeederState) {
            case REVERSING, SLOW_FEEDING, FAST_FEEDING, IDLING:
                feeder.setWantedState(Feeder.FEEDER_STATE.REVERSING);
                break;
        }
    }

    private void hoodDropping() {}


    public void setClimbSide(ClimbSide climbSide) {
        this.climbSide = climbSide;
    }

    public void setFeederControlState(FeederControlState feederControlState) {
        this.feederControlState = feederControlState;
    }

    public void setWantedSwerveState(WantedSwerveState wantedSwerveState) {
        this.wantedSwerveState = wantedSwerveState;
    }

    public void autonomousInit() {

    }

    public void teleopInit() {
        setWantedSuperState(WantedSuperState.DEFAULT);
        setFeederControlState(FeederControlState.DEFAULTING); //Not sure what to do with these two
        setWantedSwerveState(WantedSwerveState.MANUAL_DRIVING); //<-
    }
}
