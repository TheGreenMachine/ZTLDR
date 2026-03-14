package com.team1816.season.subsystems;

import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.BaseSuperstructure;
import com.team1816.lib.subsystems.Vision;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;

public class Superstructure extends BaseSuperstructure {
    private final Shooter shooter;
    private final Gatekeeper gatekeeper;
    private final Intake intake;
    private final Feeder feeder;
    private final Climber climber;

    // TODO: Reconsider this climb side stuff.
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

    private ClimbSide climbSide = ClimbSide.LEFT;

    public void setClimbSide(ClimbSide climbSide) {
        this.climbSide = climbSide;
    }

    private WantedSuperState wantedSuperState = WantedSuperState.DEFAULT;
    private ActualSuperState actualSuperState = ActualSuperState.DEFAULTING;

    private WantedSwerveState wantedSwerveState = WantedSwerveState.MANUAL_DRIVING;
    private WantedShooterState wantedShooterState = WantedShooterState.PRESET_CLOSE;
    private WantedGatekeeperState wantedGatekeeperState = WantedGatekeeperState.CLOSE;
    private WantedIntakeState wantedIntakeState = WantedIntakeState.INTAKE;
    private WantedFeederState wantedFeederState = WantedFeederState.FEED;
    private WantedClimberState wantedClimberState = WantedClimberState.STOW;

    public Superstructure(Swerve swerve, Vision vision) {
        super(swerve, vision);
        this.shooter = Singleton.CreateSubSystem(Shooter.class);
        this.gatekeeper = Singleton.CreateSubSystem(Gatekeeper.class);
        this.intake = Singleton.CreateSubSystem(Intake.class);
        this.feeder = Singleton.CreateSubSystem(Feeder.class);
        this.climber = Singleton.CreateSubSystem(Climber.class);

        GreenLogger.periodicLog("Superstructure/Wanted Super State", () -> wantedSuperState);
        GreenLogger.periodicLog("Superstructure/Actual Super State", () -> actualSuperState);
    }

    @Override
    public void periodic() {
        super.periodic();

        actualSuperState = handleStateTransitions();

        applyStates();
    }

    private ActualSuperState handleStateTransitions() {
        return switch (wantedSuperState) {
            case DEFAULT -> ActualSuperState.DEFAULTING;
            case CLIMB_L1 -> ActualSuperState.CLIMBING_L1;
            case CLIMB_L3 -> ActualSuperState.CLIMBING_L3;
        };
    }

    private void applyStates() {
        switch (actualSuperState) {
            case DEFAULTING -> defaulting();
            case CLIMBING_L1 -> climbingL1();
            case CLIMBING_L3 -> climbingL3();
        }
    }

    public void setWantedSuperState(WantedSuperState superState) {
        this.wantedSuperState = superState;
    }

    public void setSuperstructureWantedSwerveState(WantedSwerveState swerveState) {
        this.wantedSwerveState = swerveState;
    }

    public void setSuperstructureWantedShooterState(WantedShooterState shooterState) {
        this.wantedShooterState = shooterState;
    }

    public void setSuperstructureWantedGatekeeperState(WantedGatekeeperState gatekeeperState) {
        this.wantedGatekeeperState = gatekeeperState;
    }

    public void setSuperstructureWantedIntakeState(WantedIntakeState intakeState) {
        this.wantedIntakeState = intakeState;
    }

    public void setSuperstructureWantedFeederState(WantedFeederState feederState) {
        this.wantedFeederState = feederState;
    }

    public void setSuperstructureWantedClimberState(WantedClimberState climberState) {
        this.wantedClimberState = climberState;
    }

    /**
     * Sets if the incline of the shooter should duck down to fit under the trench.
     *
     * @param shouldInclineDuck If the incline of the shooter should duck down to fit under the trench.
     */
    public void setInclineDucking(boolean shouldInclineDuck) {
        shooter.setInclineDucking(shouldInclineDuck);
    }

    /**
     * Sets the angle to point the turret at when the turret isn't auto aiming (in degrees).
     *
     * @param wantedAngleDegrees The angle to point the turret at when the turret isn't auto aiming
     *                           (in degrees).
     */
    public void setTurretFixedAngle(double wantedAngleDegrees) {
        shooter.setTurretFixedAngle(wantedAngleDegrees);
    }

    /**
     * Sets if the turret should automatically point at either the hub or the corners. If false,
     * the turret will point at the angle set by {@link #setTurretFixedAngle(double)} instead.
     * Note that this will be ignored if the shooter state is {@link
     * Shooter.ShooterState#FULLY_AUTOMATIC}.
     *
     * @param shouldAutoAimTurret If the turret of the shooter should aim automatically.
     */
    public void setAutoAimTurret(boolean shouldAutoAimTurret) {
        shooter.setAutoAimTurret(shouldAutoAimTurret);
    }

    private void setWantedSubsystemStates(
        Intake.IntakeState intakeState,
        Feeder.FEEDER_STATE feederState,
        Gatekeeper.GATEKEEPER_STATE gatekeeperState,
        Shooter.ShooterState shooterState,
        Climber.CLIMBER_STATE climbState
    )  {
        intake.setWantedState(intakeState);
        feeder.setWantedState(feederState);
        gatekeeper.setWantedState(gatekeeperState);
        shooter.setWantedState(shooterState);
        climber.setWantedState(climbState);
    }

    private void defaulting() {
        // TODO: Declimb if necessary, else set the subsystem wanted states
        swerve.setWantedState(
            switch (wantedSwerveState) {
                case MANUAL_DRIVING -> Swerve.ActualState.MANUAL_DRIVING;
                case AUTOMATIC_DRIVING -> Swerve.ActualState.AUTOMATIC_DRIVING;
            }
        );
        shooter.setWantedState(
            switch (wantedShooterState) {
                case FULLY_AUTOMATIC -> Shooter.ShooterState.FULLY_AUTOMATIC;
                case PRESET_CLOSE -> Shooter.ShooterState.PRESET_CLOSE;
                case PRESET_MIDDLE -> Shooter.ShooterState.PRESET_MIDDLE;
                case PRESET_FAR -> Shooter.ShooterState.PRESET_FAR;
            }
        );
        gatekeeper.setWantedState(
            shooter.isAimed()
                ? switch (wantedGatekeeperState) {
                    case OPEN -> Gatekeeper.GATEKEEPER_STATE.OPEN;
                    case CLOSE -> Gatekeeper.GATEKEEPER_STATE.CLOSED;
                }
                : Gatekeeper.GATEKEEPER_STATE.CLOSED
        );
        intake.setWantedState(
            switch (wantedIntakeState) {
                case INTAKE -> Intake.IntakeState.INTAKE;
                case STOW -> Intake.IntakeState.STOW;
            }
        );
        feeder.setWantedState(
            switch (wantedFeederState) {
                case FEED -> Feeder.FEEDER_STATE.SLOW_FEEDING;
                case IDLE -> Feeder.FEEDER_STATE.IDLING;
            }
        );
        climber.setWantedState(
            switch (wantedClimberState) {
                case STOW, UP -> Climber.CLIMBER_STATE.IDLING;
                case L1 -> Climber.CLIMBER_STATE.L1_UP_CLIMBING;
                case L3 -> Climber.CLIMBER_STATE.L3_UP_CLIMBING;
            }
        );
    }

    private void climbingL1() {
        // TODO: Handle Superstructure climbing behavior.
        setWantedSubsystemStates(Intake.IntakeState.STOW, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.ShooterState.IDLE,
            Climber.CLIMBER_STATE.L1_UP_CLIMBING);
    }

    private void climbingL3() {
        // TODO: Handle Superstructure climbing behavior.
        setWantedSubsystemStates(Intake.IntakeState.STOW, Feeder.FEEDER_STATE.SLOW_FEEDING,
            Gatekeeper.GATEKEEPER_STATE.CLOSED, Shooter.ShooterState.IDLE,
            Climber.CLIMBER_STATE.L3_UP_CLIMBING);
    }

    public enum WantedSuperState {
        DEFAULT,
        CLIMB_L1,
        CLIMB_L3
    }

    public enum ActualSuperState {
        DEFAULTING,
        CLIMBING_L1,
        CLIMBING_L3
    }

    public enum WantedSwerveState {
        MANUAL_DRIVING,
        AUTOMATIC_DRIVING
    }

    public enum WantedShooterState {
        FULLY_AUTOMATIC,
        PRESET_CLOSE,
        PRESET_MIDDLE,
        PRESET_FAR
    }

    public enum WantedGatekeeperState {
        OPEN,
        CLOSE
    }

    public enum WantedIntakeState {
        INTAKE,
        STOW
    }

    public enum WantedFeederState {
        FEED,
        IDLE
    }

    public enum WantedClimberState {
        STOW,
        UP,
        L1,
        L3
    }
}
