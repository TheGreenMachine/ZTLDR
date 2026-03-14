package com.team1816.season.subsystems;

import com.team1816.lib.BaseRobotState;
import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.BaseSuperstructure;
import com.team1816.lib.subsystems.Vision;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//Some wantedStates are still here since they're tied to aspects of the code I don't 100% understand(ClimbSide & FeederControl) -Ishwaq
public class Superstructure extends BaseSuperstructure {
    private final Shooter shooter;
    private final Gatekeeper gatekeeper;
    private final Intake intake;
    private final Feeder feeder;
    private final Climber climber;

    private CommandXboxController controller;

    public enum WantedSuperState {
        DEFAULT,

        INITIALIZING,

        SHOOTER_AUTOMATIC_HUB,

        SHOOTER_DISTANCE_1,
        SHOOTER_DISTANCE_2,
        SHOOTER_DISTANCE_3,

        SNOWBLOWER_AUTOMATIC_CORNER,

        INTAKE_IN_AND_OFF,
        INTAKE_OUT_AND_ON,

        GATEKEEPER_ON,
        GATEKEEPER_OFF,

        CLIMB_L1,
        CLIMB_L3,
        CLIMB_DOWN_L1,

        DROP_HEIGHT
    }

    public enum ActualSuperState {
        DEFAULTING,

        INITIALIZING,

        SHOOTING_AUTOMATIC_HUB,

        SHOOTING_DISTANCE_1,
        SHOOTING_DISTANCE_2,
        SHOOTING_DISTANCE_3,

        SNOWBLOWING_AUTOMATIC_CORNER,

        INTAKING_IN_AND_OFF,
        INTAKING_OUT_AND_ON,

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

    private WantedSuperState wantedSuperState = WantedSuperState.INITIALIZING;
    private WantedSuperState previousWantedSuperState = WantedSuperState.INITIALIZING;
    private ActualSuperState actualSuperState = ActualSuperState.INITIALIZING;

    private WantedSwerveState wantedSwerveState = WantedSwerveState.MANUAL_DRIVING; //Do we need this??
    private FeederControlState feederControlState = FeederControlState.DEFAULTING; //What to do with this?
    private boolean isAutonomous = false;


    private ClimbSide climbSide = ClimbSide.LEFT;

    public Superstructure(Swerve swerve, Vision vision) {
        super(swerve, vision);
        this.shooter = Singleton.CreateSubSystem(Shooter.class);
        this.gatekeeper = Singleton.CreateSubSystem(Gatekeeper.class);
        this.intake = Singleton.CreateSubSystem(Intake.class);
        this.feeder = Singleton.CreateSubSystem(Feeder.class);
        this.climber = Singleton.CreateSubSystem(Climber.class);
    }

    @Override
    public void periodic() {
        super.periodic();

        actualSuperState = handleStateTransitions();

        applyStates();

        if (wantedSuperState != previousWantedSuperState) {
            GreenLogger.log("Wanted Superstate " + wantedSuperState);
            GreenLogger.log("Actual Superstate " + actualSuperState);
            //SmartDashboard.putString("Super state: ", wantedSuperState.toString());
            previousWantedSuperState = wantedSuperState;
        }
    }

    private ActualSuperState handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT -> actualSuperState = ActualSuperState.DEFAULTING;

            case INITIALIZING -> actualSuperState = ActualSuperState.INITIALIZING;

            case SHOOTER_AUTOMATIC_HUB -> actualSuperState = ActualSuperState.SHOOTING_AUTOMATIC_HUB;

            case SHOOTER_DISTANCE_1 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_1;
            case SHOOTER_DISTANCE_2 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_2;
            case SHOOTER_DISTANCE_3 -> actualSuperState = ActualSuperState.SHOOTING_DISTANCE_3;

            case INTAKE_OUT_AND_ON -> actualSuperState = ActualSuperState.INTAKING_OUT_AND_ON;
            case INTAKE_IN_AND_OFF -> actualSuperState = ActualSuperState.INTAKING_IN_AND_OFF;

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

            case INITIALIZING -> initializing();

            case SHOOTING_AUTOMATIC_HUB -> shootingAutomaticHub();

            case SHOOTING_DISTANCE_1 -> shootingDistance1();
            case SHOOTING_DISTANCE_2 -> shootingDistance2();
            case SHOOTING_DISTANCE_3 -> shootingDistance3();

            case SNOWBLOWING_AUTOMATIC_CORNER -> snowblowingAutomaticCorner();

            case INTAKING_OUT_AND_ON -> intakeOutAndOn();
            case INTAKING_IN_AND_OFF -> intakeInAndOff();

            case GATEKEEPING_ON -> gatekeepingOn();
            case GATEKEEPING_OFF -> gatekeeperingOff();

            case CLIMBING_L1 -> climbingL1();
            case CLIMBING_L3 -> climbingL3();
            case CLIMBING_DOWN_L1 -> climbingDownL1();

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

    private void defaulting() {
        // During auto, let PathPlanner have sole control of the drivetrain
        if (!isAutonomous) {
            swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
        }
    }

    private void initializing() {
        shooter.setWantedState(Shooter.SHOOTER_STATE.CALIBRATING);
        if (shooter.isCalibrated()) {
            setWantedSuperState(WantedSuperState.DEFAULT);
        }
    }

    private void shootingAutomaticHub() {
        if(gatekeeper.getWantedState() == Gatekeeper.GATEKEEPER_STATE.CLOSED) {
            shooter.setWantedState(Shooter.SHOOTER_STATE.AIMING_HUB);
        } else {
            shooter.setWantedState(Shooter.SHOOTER_STATE.AUTOMATIC);
        }
//        actualSuperState = ActualSuperState.DEFAULTING;
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
        if(gatekeeper.getWantedState() == Gatekeeper.GATEKEEPER_STATE.CLOSED) {
            shooter.setWantedState(Shooter.SHOOTER_STATE.AIMING_CORNER);
        } else {
            shooter.setWantedState(Shooter.SHOOTER_STATE.SNOWBLOWING);
        }
    }

    private void intakeOutAndOn() {
        intake.setWantedState(Intake.INTAKE_STATE.INTAKE_OUT_AND_ON);
        feeder.setWantedState(Feeder.FEEDER_STATE.SLOW_FEEDING);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void intakeInAndOff() {
        intake.setWantedState(Intake.INTAKE_STATE.INTAKE_IN_AND_OFF);
        feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void gatekeepingOn() {
        if(shooter.getWantedState() == Shooter.SHOOTER_STATE.AIMING_HUB) {
            shooter.setWantedState(Shooter.SHOOTER_STATE.AUTOMATIC);
        } else if (shooter.getWantedState() == Shooter.SHOOTER_STATE.AIMING_CORNER) {
            shooter.setWantedState(Shooter.SHOOTER_STATE.SNOWBLOWING);
        }
        gatekeeper.setWantedState(Gatekeeper.GATEKEEPER_STATE.OPEN);
        feeder.setWantedState(Feeder.FEEDER_STATE.FAST_FEEDING);
        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void gatekeeperingOff() {
        if(shooter.getWantedState() == Shooter.SHOOTER_STATE.AUTOMATIC) {
            shooter.setWantedState(Shooter.SHOOTER_STATE.AIMING_HUB);
        } else if (shooter.getWantedState() == Shooter.SHOOTER_STATE.SNOWBLOWING) {
            shooter.setWantedState(Shooter.SHOOTER_STATE.AIMING_CORNER);
        }
        gatekeeper.setWantedState(Gatekeeper.GATEKEEPER_STATE.CLOSED);

        if (intake.isIntaking()) {
            feeder.setWantedState(Feeder.FEEDER_STATE.FAST_FEEDING);
        } else {
            feeder.setWantedState(Feeder.FEEDER_STATE.SLOW_FEEDING);
        }

        actualSuperState = ActualSuperState.DEFAULTING;
    }

    private void climbingL1() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_IN_AND_OFF, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.L1_UP_CLIMBING);
    }

    private void climbingL3() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_IN_AND_OFF, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.L3_UP_CLIMBING);
    }

    private void climbingDownL1() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_IN_AND_OFF, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.L1_DOWN_CLIMBING);
    }

    private void droppingHeight() {
        setWantedSubsystemStates(Intake.INTAKE_STATE.INTAKE_IN_AND_OFF, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.SHOOTER_STATE.IDLE,
            Climber.CLIMBER_STATE.IDLING);
    }

    public void setClimbSide(ClimbSide climbSide) {
        this.climbSide = climbSide;
    }

    public void autonomousInit() {
        isAutonomous = true;
        swerve.setWantedState(Swerve.ActualState.IDLING);
    }

    public void teleopInit() {
        isAutonomous = false;
        swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
        intake.setWantedState(Intake.INTAKE_STATE.INTAKE_OUT_AND_ON);
        feeder.setWantedState(Feeder.FEEDER_STATE.SLOW_FEEDING);
        shooter.setWantedState(Shooter.SHOOTER_STATE.DISTANCE_ONE);
        gatekeeper.setWantedState(Gatekeeper.GATEKEEPER_STATE.CLOSED);
    }

    public void shootingAutomatic() {  //the shooter position to automatically shoot at hub or corner depending on location
        if (BaseRobotState.robotPose.getX() >  5) {  //TODO: account for color and fix the number
            setWantedSuperState(WantedSuperState.SNOWBLOWER_AUTOMATIC_CORNER);
        } else {
            setWantedSuperState(WantedSuperState.SHOOTER_AUTOMATIC_HUB);
        }
    }

    public void incrementFlipperInwards() {
        intake.incrementFlipperInwards();
    }

    public void resetFlipperOut() {
        intake.resetFlipperOut();
    }
}
