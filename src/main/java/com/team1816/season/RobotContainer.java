package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.BaseRobotContainer;
import com.team1816.season.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer extends BaseRobotContainer {
    private Superstructure superstructure;

    public RobotContainer() {
        NamedCommands.registerCommand("InTheZone", new InTheZoneCommand());
        // call the base to initialize library objects
        // i.e. subsystems that always exist like the drivetrain and path planner
        initializeLibSubSystems();

        configureBindings();
        registerCommands();
    }

    @Override
    protected Superstructure createSuperstructure() {
        superstructure = new Superstructure(swerve, vision);
        return superstructure;
    }

    public void autonomousInit() {
        superstructure.setWantedSuperState(Superstructure.WantedSuperState.DEFAULT);
        // TODO: This may or may not be what we want for PathPlanner path following.
        superstructure.setSuperstructureWantedSwerveState(Superstructure.WantedSwerveState.AUTOMATIC_DRIVING);
        superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC);
        superstructure.setInclineDucking(false);
        superstructure.setTurretPresetAngle(0);
        superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE);
        superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.INTAKE);
        superstructure.setSuperstructureWantedFeederState(Superstructure.WantedFeederState.FEED);
        superstructure.setSuperstructureWantedClimberState(Superstructure.WantedClimberState.STOW);
    }

    public void teleopInit() {
        superstructure.setWantedSuperState(Superstructure.WantedSuperState.DEFAULT);
        superstructure.setSuperstructureWantedSwerveState(Superstructure.WantedSwerveState.MANUAL_DRIVING);
        superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC);
        superstructure.setTurretPresetAngle(0);
        superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE);
    }

    private void configureBindings() {
        // TODO: Set up the controls that we actually want.
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
        driverController.a().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC)));

        // Intake
        driverController.leftBumper().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.INTAKE)));
        driverController.rightBumper().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.STOW)));
    }

    public final void registerCommands() {
        // TODO: Set up the PathPlanner NamedCommands that we actually want.
        // Shooter
        NamedCommands.registerCommand("shooter/automatic", Commands.runOnce(() ->
             superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC)
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
        NamedCommands.registerCommand("shooter/setTurretPresetAngle0Degrees", Commands.runOnce(() ->
            superstructure.setTurretPresetAngle(0)
        ));
        NamedCommands.registerCommand("shooter/setTurretPresetAngle90Degrees", Commands.runOnce(() ->
            superstructure.setTurretPresetAngle(90)
        ));
        NamedCommands.registerCommand("shooter/setTurretPresetAngle180Degrees", Commands.runOnce(() ->
            superstructure.setTurretPresetAngle(180)
        ));
        NamedCommands.registerCommand("shooter/setTurretPresetAngle270Degrees", Commands.runOnce(() ->
            superstructure.setTurretPresetAngle(270)
        ));

        NamedCommands.registerCommand("intake/intake", Commands.runOnce(() ->
             superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.INTAKE)
        ));

        // Gatekeeper
        NamedCommands.registerCommand("gatekeeper/open", Commands.runOnce(() ->
            superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN)
        ));
        NamedCommands.registerCommand("gatekeeper/close", Commands.runOnce(() ->
            superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE)
        ));

        // Superstructure
        NamedCommands.registerCommand("climbL1", Commands.runOnce(() ->
            superstructure.setWantedSuperState(Superstructure.WantedSuperState.CLIMB_L1)
        ));
        NamedCommands.registerCommand("default", Commands.runOnce(() ->
            superstructure.setWantedSuperState(Superstructure.WantedSuperState.DEFAULT)
        ));
    }
}
