package com.team1816.season.subsystems;

import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.BaseSuperstructure;
import com.team1816.lib.subsystems.Vision;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import com.team1816.lib.util.GreenLogger;

public class Superstructure extends BaseSuperstructure {
    private final Shooter shooter;
    private final Gatekeeper gatekeeper;
    private final Intake intake;
    private final Feeder feeder;

    private WantedSuperState wantedSuperState = WantedSuperState.DEFAULT;
    private ActualSuperState actualSuperState = ActualSuperState.DEFAULTING;

    private WantedSwerveState wantedSwerveState = WantedSwerveState.MANUAL_DRIVING;
    private WantedShooterState wantedShooterState = WantedShooterState.PRESET_CLOSE;
    private WantedGatekeeperState wantedGatekeeperState = WantedGatekeeperState.CLOSE;
    private WantedIntakeState wantedIntakeState = WantedIntakeState.INTAKE;
    private WantedFeederState wantedFeederState = WantedFeederState.FEED;

    /**
     * If the gatekeeper should directly accept control from outside without first checking that
     * the shooter is ready. This is so we can still shoot if something is wrong with the shooter,
     * and it thinks it is never aimed.
     */
    private boolean forceAllowGatekeeperControl = false;

    public Superstructure(Swerve swerve, Vision vision) {
        super(swerve, vision);
        this.shooter = Singleton.CreateSubSystem(Shooter.class);
        this.gatekeeper = Singleton.CreateSubSystem(Gatekeeper.class);
        this.intake = Singleton.CreateSubSystem(Intake.class);
        this.feeder = Singleton.CreateSubSystem(Feeder.class);

        GreenLogger.periodicLog("Superstructure/Wanted Super State", () -> wantedSuperState);
        GreenLogger.periodicLog("Superstructure/Actual Super State", () -> actualSuperState);
        GreenLogger.periodicLog("Superstructure/Force Allowing Gatekeeper Control", () -> forceAllowGatekeeperControl);
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

    public void incrementPullInSuperstructureIntakeState() {
        wantedIntakeState = switch (wantedIntakeState) {
            case INTAKE -> WantedIntakeState.PULL_IN_ONE;
            case PULL_IN_ONE -> WantedIntakeState.PULL_IN_TWO;
            case PULL_IN_TWO, STOW -> WantedIntakeState.STOW;
        };
    }

    public void setSuperstructureWantedFeederState(WantedFeederState feederState) {
        this.wantedFeederState = feederState;
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

    /**
     * Sets if the gatekeeper should directly accept control from outside without first checking
     * that the shooter is ready. This is so we can still shoot if something is wrong with the
     * shooter, and it thinks it is never aimed.
     *
     * @param shouldForceAllowGatekeeperControl If the gatekeeper should accept controls without
     *                                          waiting for the shooter to be ready.
     */
    public void forceAllowGatekeeperControl(boolean shouldForceAllowGatekeeperControl) {
        forceAllowGatekeeperControl = shouldForceAllowGatekeeperControl;
    }

    /**
     * Increases the adjustment value to all requests to the launch motors.
     */
    public void increaseLaunchVelocityAdjustment() {
        shooter.increaseLaunchVelocityAdjustment();
    }

    /**
     * Decreases the adjustment value to all requests to the launch motors.
     */
    public void decreaseLaunchVelocityAdjustment() {
        shooter.decreaseLaunchVelocityAdjustment();
    }

    /**
     * Increases the adjustment value to all requests to the incline.
     */
    public void increaseInclineAngleAdjustment() {
        shooter.increaseInclineAngleAdjustment();
    }

    /**
     * Decreases the adjustment value to all requests to the incline.
     */
    public void decreaseInclineAngleAdjustment() {
        shooter.decreaseInclineAngleAdjustment();
    }

    /**
     * Increases the adjustment value to all requests to the turret.
     */
    public void increaseTurretAngleAdjustment() {
        shooter.increaseTurretAngleAdjustment();
    }

    /**
     * Decreases the adjustment value to all requests to the turret.
     */
    public void decreaseTurretAngleAdjustment() {
        shooter.decreaseTurretAngleAdjustment();
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
            // Only let fuel into the shooter if the shooter is ready, or if we are force allowing
            // control.
            shooter.isAimed() || forceAllowGatekeeperControl
                ? switch (wantedGatekeeperState) {
                    case OPEN -> Gatekeeper.GatekeeperState.OPEN;
                    case CLOSE -> Gatekeeper.GatekeeperState.CLOSED;
                }
                : Gatekeeper.GatekeeperState.CLOSED
        );
        // Only spin up the launch motors if we are trying to shoot to save battery.
        shooter.setSpinUpLaunchMotors(gatekeeper.getState() == Gatekeeper.GatekeeperState.OPEN);
        intake.setWantedState(
            switch (wantedIntakeState) {
                case INTAKE -> Intake.IntakeState.INTAKE;
                case STOW -> Intake.IntakeState.STOW;
                case PULL_IN_ONE -> Intake.IntakeState.PULL_IN_ONE;
                case PULL_IN_TWO -> Intake.IntakeState.PULL_IN_TWO;
            }
        );
        feeder.setWantedState(
            gatekeeper.getState() == Gatekeeper.GatekeeperState.OPEN
                ? switch (wantedFeederState) {
                    case FEED -> Feeder.FeederState.FEEDING;
                    case STOP -> Feeder.FeederState.STOPPED;
                }
                // If the gatekeeper isn't running, keep the feeder stopped to avoid jamming.
                : Feeder.FeederState.STOPPED
        );
    }

    private void climbingL1() {
        // TODO: Handle Superstructure climbing behavior.
    }

    private void climbingL3() {
        // TODO: Handle Superstructure climbing behavior.
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
        STOW,
        PULL_IN_ONE,
        PULL_IN_TWO
    }

    public enum WantedFeederState {
        FEED,
        STOP
    }
}
