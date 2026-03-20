package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.BaseRobotContainer;
import com.team1816.lib.BaseRobotState;
import com.team1816.lib.util.GreenLogger;
import com.team1816.season.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer extends BaseRobotContainer {
    private Superstructure superstructure;

    public RobotContainer() {
        NamedCommands.registerCommand("InTheZone", new InTheZoneCommand());

        configureBindings();
        registerCommands();

        // call the base to initialize library objects
        // i.e. subsystems that always exist like the drivetrain and path planner
        initializeLibSubSystems();
    }

    @Override
    protected Superstructure createSuperstructure() {
        superstructure = new Superstructure(swerve, vision);
        return superstructure;
    }

    public void autonomousInit() {
        superstructure.setWantedSuperState(Superstructure.WantedSuperState.DEFAULT);
        superstructure.setSuperstructureWantedSwerveState(Superstructure.WantedSwerveState.AUTOMATIC_DRIVING);
        superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterDistanceState.AUTOMATIC);
        superstructure.setInclineDucking(true);
        superstructure.setTurretFixedAngle(0);
        superstructure.setAutoAimTurret(true);
        superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE);
        superstructure.setGatekeeperAndFeederReversing(false);
        superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.INTAKE);
        superstructure.forceAllowGatekeeperControl(true);
    }

    public void teleopInit() {
        superstructure.setWantedSuperState(Superstructure.WantedSuperState.DEFAULT);
        superstructure.setSuperstructureWantedSwerveState(Superstructure.WantedSwerveState.MANUAL_DRIVING);
        superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterDistanceState.AUTOMATIC);
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

        driverController.povLeft().onTrue(Commands.runOnce(() -> BaseRobotState.hasAccuratePoseEstimate = false));

        // Gatekeeper
        driverController.rightTrigger().onTrue(Commands.runOnce(() -> {
            superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN);
            superstructure.setInclineDucking(false);
        }))
            .onFalse(Commands.runOnce(() -> {
                superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE);
                superstructure.setInclineDucking(true);
            }));

        driverController.povUp().onTrue(Commands.runOnce(() ->
                superstructure.setGatekeeperAndFeederReversing(true)
            ))
            .onFalse(Commands.runOnce(() ->
                superstructure.setGatekeeperAndFeederReversing(false)
            ));

        // Shooter
        driverController.a().onTrue(Commands.runOnce(() -> {
            superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterDistanceState.PRESET_CLOSE);
            superstructure.setTurretFixedAngle(0);
            superstructure.setAutoAimTurret(false);
        }));
        driverController.b().onTrue(Commands.runOnce(() -> {
            superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterDistanceState.PRESET_MIDDLE);
            superstructure.setTurretFixedAngle(0);
            superstructure.setAutoAimTurret(false);
        }));
        driverController.y().onTrue(Commands.runOnce(() -> {
            superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterDistanceState.PRESET_FAR);
            superstructure.setTurretFixedAngle(0);
            superstructure.setAutoAimTurret(false);
        }));
        driverController.x().onTrue(Commands.runOnce(() -> {
            superstructure.setSuperstructureWantedShooterState(Superstructure.WantedShooterDistanceState.AUTOMATIC);
            superstructure.setAutoAimTurret(true);
        }));

        // Intake
        driverController.leftBumper().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.INTAKE)));
        driverController.rightBumper().onTrue(Commands.runOnce(() -> superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.STOW)));


        // OPERATOR CONTROLLER
        // Gatekeeper
        operatorController.x().onTrue(Commands.runOnce(() ->
            superstructure.forceAllowGatekeeperControl(false))
        );
        operatorController.y().onTrue(Commands.runOnce(() ->
            superstructure.forceAllowGatekeeperControl(true))
        );

        // Shooter
        operatorController.povDown().onTrue(Commands.runOnce(() -> superstructure.recalibrateTurret()));


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
        NamedCommands.registerCommand("shoot", Commands.sequence(
            Commands.runOnce(() -> GreenLogger.log("Running named command: shoot")),
            Commands.runOnce(() -> superstructure.setInclineDucking(false)),
            Commands.runOnce(() -> superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN)),
            Commands.race(
                Commands.repeatingSequence(
                    Commands.runOnce(() -> superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.STOW)),
                    Commands.waitSeconds(0.3),
                    Commands.runOnce(() -> superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.INTAKE)),
                    Commands.waitSeconds(0.3)
                ),
                Commands.waitSeconds(5)
            ),
            Commands.runOnce(() -> superstructure.setSuperstructureWantedIntakeState(Superstructure.WantedIntakeState.INTAKE)),
            Commands.runOnce(() -> superstructure.setInclineDucking(true)),
            Commands.runOnce(() -> superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSE))
        ));

        NamedCommands.registerCommand("waitForDucking", Commands.parallel(
            Commands.runOnce(() -> GreenLogger.log("Running named command: waitForDucking")),
            Commands.runOnce(() -> superstructure.setInclineDucking(true)),
            Commands.waitUntil(superstructure::isInclineDucked)
        ));

        NamedCommands.registerCommand("duck", Commands.parallel(
            Commands.runOnce(() -> GreenLogger.log("Running named command: duck")),
            Commands.runOnce(() -> superstructure.setInclineDucking(true))
        ));

        NamedCommands.registerCommand("unduck", Commands.parallel(
            Commands.runOnce(() -> GreenLogger.log("Running named command: unduck")),
            Commands.runOnce(() -> superstructure.setInclineDucking(false))
        ));

        NamedCommands.registerCommand("gatekeeper/open", Commands.parallel(
            Commands.runOnce(() -> GreenLogger.log("Running named command: gatekeeper/open")),
            Commands.runOnce(() ->
                superstructure.setSuperstructureWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN)
            )
        ));

        NamedCommands.registerCommand("fixTurretAngle180", Commands.parallel(
            Commands.runOnce(() -> GreenLogger.log("Running named command: fixTurretAngle180")),
            Commands.runOnce(() -> superstructure.setAutoAimTurret(false)),
            Commands.runOnce(() -> superstructure.setTurretFixedAngle(180))
        ));
    }
}
