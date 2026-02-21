package com.team1816.season.subsystems;

import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public Command setStateCommand(WantedSuperState superState) {
        return new InstantCommand(() -> setWantedSuperState(superState));
    }
    private void defaulting() {
        /*
         * What is this doing???
         */
        if (feederControlState == FeederControlState.OVERRIDING) {
            feeder.setWantedState(Feeder.FEEDER_STATE.FAST_FEEDING);
        }
        else {
            if (wantedIntakeState == Intake.INTAKE_STATE.INTAKE_IN || wantedGatekeeperState == Gatekeeper.GATEKEEPER_STATE.OPEN) {
                feeder.setWantedState(Feeder.FEEDER_STATE.SLOW_FEEDING);
            }
            else {
                feeder.setWantedState(Feeder.FEEDER_STATE.IDLING);
            }
        }

        swerve.setWantedState(Swerve.ActualState.MANUAL_DRIVING);
    }
    private void shootCalibrating() {
        wantedShooterState = Shooter.SHOOTER_STATE.CALIBRATING;
    }
    private void shootingAutomaticHub() {
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.OPEN;
        wantedFeederState = Feeder.FEEDER_STATE.FAST_FEEDING;
        wantedShooterState = Shooter.SHOOTER_STATE.AUTOMATIC;  //Figure out how this is going to work
    }
    private void shootingAutomaticCorner() {
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.OPEN;
        wantedFeederState = Feeder.FEEDER_STATE.FAST_FEEDING;
        wantedShooterState = Shooter.SHOOTER_STATE.AUTOMATIC;  //Figure out how this is going to work
    }
    private void shootingDistance1() {
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.OPEN;
        wantedFeederState = Feeder.FEEDER_STATE.FAST_FEEDING;
        wantedShooterState = Shooter.SHOOTER_STATE.DISTANCE_ONE;
    }
    private void shootingDistance2() {
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.OPEN;
        wantedFeederState = Feeder.FEEDER_STATE.FAST_FEEDING;
        wantedShooterState = Shooter.SHOOTER_STATE.DISTANCE_TWO;
    }
    private void shootingDistance3() {
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.OPEN;
        wantedFeederState = Feeder.FEEDER_STATE.FAST_FEEDING;
        wantedShooterState = Shooter.SHOOTER_STATE.DISTANCE_THREE;
    }
    private void snowblowingAutomaticHub() {
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_IN;
        wantedFeederState = Feeder.FEEDER_STATE.FAST_FEEDING;
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.OPEN;
        wantedShooterState = Shooter.SHOOTER_STATE.AUTOMATIC; //Figure out how this is going to work
    }
    private void snowblowingAutomaticCorner() {
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_IN;
        wantedFeederState = Feeder.FEEDER_STATE.FAST_FEEDING;
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.OPEN;
        wantedShooterState = Shooter.SHOOTER_STATE.AUTOMATIC; //Figure out how this is going to work
    }
    private void snowblowingDistance1() {
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_IN;
        wantedFeederState = Feeder.FEEDER_STATE.FAST_FEEDING;
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.OPEN;
        wantedShooterState = Shooter.SHOOTER_STATE.DISTANCE_ONE;
    }
    private void snowblowingDistance2() {
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_IN;
        wantedFeederState = Feeder.FEEDER_STATE.FAST_FEEDING;
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.OPEN;
        wantedShooterState = Shooter.SHOOTER_STATE.DISTANCE_TWO;
    }
    private void snowblowingDistance3() {
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_IN;
        wantedFeederState = Feeder.FEEDER_STATE.FAST_FEEDING;
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.OPEN;
        wantedShooterState = Shooter.SHOOTER_STATE.DISTANCE_THREE;
    }
    private void intakeLifting() {
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_UP;
    }
    private void intakeDropping() {
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_DOWN;
    }
    private void intaking() {
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_IN;
    }
    private void outtaking() {
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.CLOSED; //Is this okay?
        wantedFeederState = Feeder.FEEDER_STATE.REVERSING;
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_OUT;
    }
    private void gatekeepingOn() {
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.OPEN;
    }
    private void gatekeeperingOff() {
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.CLOSED;
    }
    private void climbingL1() {
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_DOWN; //Will need to ask build team about this
        wantedFeederState = Feeder.FEEDER_STATE.IDLING;
        wantedShooterState = Shooter.SHOOTER_STATE.IDLE;
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.CLOSED;
        wantedClimbState = Climber.CLIMBER_STATE.L1_UP_CLIMBING;
    }
    private void climbingL3() {
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_DOWN; //Will need to ask build team about this
        wantedFeederState = Feeder.FEEDER_STATE.IDLING;
        wantedShooterState = Shooter.SHOOTER_STATE.IDLE;
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.CLOSED;
        wantedClimbState = Climber.CLIMBER_STATE.L3_UP_CLIMBING;
    }
    private void climbingDownL1() {
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_DOWN; //Will need to ask build team about this
        wantedFeederState = Feeder.FEEDER_STATE.IDLING;
        wantedShooterState = Shooter.SHOOTER_STATE.IDLE;
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.CLOSED;
        wantedClimbState = Climber.CLIMBER_STATE.L1_DOWN_CLIMBING;
    }
    private void feedingSlow() {
        wantedFeederState = Feeder.FEEDER_STATE.SLOW_FEEDING;
    }
    private void feedingFast() {
        wantedFeederState = Feeder.FEEDER_STATE.FAST_FEEDING;
    }
    private void agitate() {
        switch(wantedFeederState) {
            case REVERSING, SLOW_FEEDING, FAST_FEEDING, IDLING:
                feeder.setWantedState(Feeder.FEEDER_STATE.REVERSING);
                break;
        }
    }
    private void droppingHeight() {
        wantedGatekeeperState = Gatekeeper.GATEKEEPER_STATE.CLOSED;
        wantedShooterState = Shooter.SHOOTER_STATE.IDLE; //Will this work??
        wantedIntakeState = Intake.INTAKE_STATE.INTAKE_DOWN;
    }


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
            //What is supposed to be here???
    }

    public void teleopInit() {
        setWantedSuperState(WantedSuperState.DEFAULT); //We're doing this twice...
        setFeederControlState(FeederControlState.DEFAULTING); //Not sure what to do with this
        setWantedSwerveState(WantedSwerveState.MANUAL_DRIVING); //<-ditto
    }
}
