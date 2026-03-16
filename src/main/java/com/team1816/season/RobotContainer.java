package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.BaseRobotContainer;
import com.team1816.lib.BaseRobotState;
import com.team1816.season.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer extends BaseRobotContainer {
    private Superstructure superstructure;

    public RobotContainer() {
        NamedCommands.registerCommand("InTheZone", new InTheZoneCommand());
        // call the base to initialize library objects
        // i.e. subsystems that always exist like the drivetrain and path planner
        initializeLibSubSystems();

        buildAutoChooser();

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
        superstructure.setSuperstructureWantedSwerveState(Superstructure.WantedSwerveState.AUTOMATIC_DRIVING);
        superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_CLOSE);
        superstructure.setInclineDucking(true);
        superstructure.setTurretFixedAngle(0);
        superstructure.setAutoAimTurret(true);
        superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE);
        superstructure.setGatekeeperAndFeederReversing(false);
        superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.INTAKE);
    }

    public void teleopInit() {
        superstructure.setWantedSuperState(Superstructure.WantedSuperState.DEFAULT);
        superstructure.setSuperstructureWantedSwerveState(Superstructure.WantedSwerveState.MANUAL_DRIVING);
        superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_CLOSE);
        superstructure.setInclineDucking(true);
        superstructure.setTurretFixedAngle(0);
        superstructure.setAutoAimTurret(true);
        superstructure.setGatekeeperAndFeederReversing(false);
        superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE);
    }

    private void configureBindings() {
        // DRIVER CONTROLLER
        // Swerve
        driverController.povDown().onTrue(Commands.runOnce(() ->
            superstructure.setSuperstructureWantedSwerveState(Superstructure.WantedSwerveState.BRAKE)
        ))
            .onFalse(Commands.runOnce(() ->
                superstructure.setSuperstructureWantedSwerveState(Superstructure.WantedSwerveState.MANUAL_DRIVING)
            ));

        // Gatekeeper
        driverController.rightTrigger().onTrue(Commands.runOnce(() -> {
            superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN);
            superstructure.setInclineDucking(false);
        }))
            .onFalse(Commands.runOnce(() -> {
                superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE);
                superstructure.setInclineDucking(true);
            }));
        driverController.povUp().onTrue(Commands.runOnce(() -> BaseRobotState.hasAccuratePoseEstimate = false));

        // Shooter
        driverController.x().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_CLOSE)));
        driverController.y().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_MIDDLE)));
        driverController.b().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.PRESET_FAR)));
        // TODO: Verify that auto distance calculations (using the lookup table) actually work. If they don't, we can just remove this control for now.
        driverController.a().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterState.FULLY_AUTOMATIC)));

        // Intake
        driverController.leftBumper().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.INTAKE)));
        driverController.rightBumper().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.STOW)));
        // For now, we just plan on retracting all the way, but we can add this back if we need it.
//        driverController.povDown().onTrue(Commands.runOnce(() -> superstructure.incrementPullInSuperstructureIntakeState()));


        // OPERATOR CONTROLLER
        // Gatekeeper
        // TODO: Put these on the buttons we actually want, or remove this feature if we don't
        //  think it is necessary. We should only have to use these if something is broken with
        //  the shooter aiming or determining when it is aimed correctly.
        operatorController.x().onTrue(Commands.runOnce(() ->
            superstructure.forceAllowGatekeeperControl(false))
        );
        operatorController.y().onTrue(Commands.runOnce(() ->
            superstructure.forceAllowGatekeeperControl(true))
        );
        operatorController.rightTrigger().onTrue(Commands.runOnce(() ->
            superstructure.setGatekeeperAndFeederReversing(true)
        ))
            .onFalse(Commands.runOnce(() ->
                superstructure.setGatekeeperAndFeederReversing(false)
            ));

        // Shooter
        // TODO: Put these on the buttons we actually want. We should only have to use these if something is broken with the auto turret aiming.
        operatorController.a().onTrue(Commands.runOnce(() -> superstructure.setAutoAimTurret(true)));
        operatorController.b().onTrue(Commands.runOnce(() -> {
            superstructure.setTurretFixedAngle(0);
            superstructure.setAutoAimTurret(false);
        }));


        // BUTTON BOARD
        // Shooter
        buttonBoard.middleLeft().whileTrue(Commands.run(() -> superstructure.increaseLaunchVelocityAdjustment()));
        buttonBoard.bottomLeft().whileTrue(Commands.run(() -> superstructure.decreaseLaunchVelocityAdjustment()));
        buttonBoard.middleCenter().whileTrue(Commands.run(() -> superstructure.increaseInclineAngleAdjustment()));
        buttonBoard.bottomCenter().whileTrue(Commands.run(() -> superstructure.decreaseInclineAngleAdjustment()));
        buttonBoard.middleRight().whileTrue(Commands.run(() -> superstructure.increaseTurretAngleAdjustment()));
        buttonBoard.bottomRight().whileTrue(Commands.run(() -> superstructure.decreaseTurretAngleAdjustment()));
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
    }
}
