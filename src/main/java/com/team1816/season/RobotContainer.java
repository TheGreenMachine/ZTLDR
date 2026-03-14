package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.BaseRobotContainer;
import com.team1816.season.subsystems.*;
import com.team1816.season.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer extends BaseRobotContainer {
    private Superstructure superstructure;

    public RobotContainer() {
        NamedCommands.registerCommand("InTheZone", new InTheZoneCommand());
        // call the base to initialize library objects
        // i.e. subsystems that always exist like the drivetrain and path planner
        initializeLibSubSystems();

        registerCommands();

        buildAutoChooser();

        configureBindings();
    }

    @Override
    protected Superstructure createSuperstructure() {
        superstructure = new Superstructure(swerve, vision);
        return superstructure;
    }

    public void autonomousInit() {
        superstructure.setWantedSuperState(Superstructure.WantedSuperState.DEFAULT);
        superstructure.setSuperstructureWantedSwerveState(Superstructure.WantedSwerveState.AUTOMATIC_DRIVING);
        superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_CLOSE);
        superstructure.setInclineDucking(false);
        superstructure.setTurretFixedAngle(0);
        superstructure.setAutoAimTurret(true);
        superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE);
        superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.INTAKE);
        superstructure.setSuperstructureWantedFeederState(Superstructure.WantedFeederState.FEED);
        superstructure.setSuperstructureWantedClimberState(Superstructure.WantedClimberState.STOW);
    }

    public void teleopInit() {
        superstructure.setWantedSuperState(Superstructure.WantedSuperState.DEFAULT);
        superstructure.setSuperstructureWantedSwerveState(Superstructure.WantedSwerveState.MANUAL_DRIVING);
        superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_CLOSE);
        superstructure.setTurretFixedAngle(0);
        superstructure.setAutoAimTurret(true);
        superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE);
        superstructure.setSuperstructureWantedFeederState(Superstructure.WantedFeederState.FEED);
    }

    private void configureBindings() {
        // Gatekeeper
        driverController.rightTrigger().onTrue(Commands.runOnce(() -> {
            superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN);
            superstructure.setInclineDucking(false);
        }))
            .onFalse(Commands.runOnce(() -> {
                superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE);
                superstructure.setInclineDucking(true);
            }));

        // Shooter
        driverController.x().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_CLOSE)));
        driverController.y().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_MIDDLE)));
        driverController.b().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_FAR)));
        // TODO: Verify that auto distance calculations (using the lookup table) actually work. If they don't, we can just remove this control for now.
        driverController.a().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.FULLY_AUTOMATIC)));

        // Intake
        driverController.leftBumper().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.INTAKE)));
        driverController.rightBumper().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.STOW)));
        driverController.povDown().onTrue(Commands.runOnce(() -> superstructure.incrementPullInSuperstructureIntakeState()));
    }

    public final void registerCommands() {
        // Gatekeeper
        NamedCommands.registerCommand("gatekeeper/open", Commands.runOnce(() ->
            superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN)
        ));
        NamedCommands.registerCommand("gatekeeper/close", Commands.runOnce(() ->
            superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE)
        ));

        // Shooter
        NamedCommands.registerCommand("shooter/automatic", Commands.runOnce(() ->
             superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.FULLY_AUTOMATIC)
        ));
        NamedCommands.registerCommand("shooter/presetClose", Commands.runOnce(() ->
            superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_CLOSE)
        ));
        NamedCommands.registerCommand("shooter/presetMiddle", Commands.runOnce(() ->
            superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_MIDDLE)
        ));
        NamedCommands.registerCommand("shooter/presetFar", Commands.runOnce(() ->
            superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_FAR)
        ));

        // BottomBlueToMiddle
        NamedCommands.registerCommand("BottomBlueToMiddleStart0", Commands.runOnce(() ->
            {
                superstructure.setInclineDucking(true);
                superstructure.setWantedSubsystemStates(
                    Intake.IntakeState.INTAKE_POSITION_2,
                    Feeder.FEEDER_STATE.FAST_FEEDING,
                    Gatekeeper.GATEKEEPER_STATE.CLOSED,
                    Shooter.ShooterState.IDLE,
                    Climber.CLIMBER_STATE.IDLING
                );
                superstructure.setTurretPresetAngle(-110);
            }
        ));
        NamedCommands.registerCommand("BottomBlueToMiddleStart1", Commands.runOnce(() ->
            superstructure.setInclineDucking(false)
        ));
        NamedCommands.registerCommand("BottomBlueToMiddleStart2", Commands.runOnce(() ->
            {
                superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN);
                superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_MIDDLE);
            }
        ));
    }
}
