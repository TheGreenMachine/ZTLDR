package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.BaseRobotContainer;
import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Feeder;
import com.team1816.season.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer extends BaseRobotContainer {
    public RobotContainer() {
        NamedCommands.registerCommand("InTheZone", new InTheZoneCommand());
        // call the base to initialize library objects
        // i.e. subsystems that always exist like the drivetrain and path planner
        initializeLibSubSystems();

        Singleton.CreateSubSystem(Feeder.class);

        superstructure = new Superstructure(swerve);

        initializeAutonomous();

        configureBindings();
        registerCommands();
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    public void autonomousInit() {
        superstructure.autonomousInit();
    }

    public void teleopInit() {
        superstructure.setWantedSuperState(Superstructure.WantedSuperState.DEFAULT);
        superstructure.teleopInit();
    }

    private void configureBindings() {
// SNOWBLOWING MODE CONTROLS:

        driverController.rightTrigger().onTrue(Commands.runOnce(() -> superstructure.toggleGatekeeper()));
        driverController.rightTrigger().onFalse(Commands.runOnce(() -> superstructure.setWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSED)));

        driverController.povUp().onTrue(Commands.runOnce(() -> superstructure.setWantedShooterState(Superstructure.WantedShooterState.DISTANCE_ONE)));
        driverController.povRight().onTrue(Commands.runOnce(() -> superstructure.setWantedShooterState(Superstructure.WantedShooterState.DISTANCE_TWO)));
        driverController.povLeft().onTrue(Commands.runOnce(() -> superstructure.setWantedShooterState(Superstructure.WantedShooterState.DISTANCE_THREE)));
        driverController.povDown().onTrue(Commands.runOnce(() -> superstructure.setWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC)));

        driverController.b().onTrue(Commands.runOnce(() -> superstructure.toggleIntake()));
        driverController.a().onTrue(Commands.runOnce(() -> superstructure.toggleHood()));
        //Agitate button TBD, would be another toggle

        //add manual shooter speed adjustments

    }

    public final void registerCommands() {
        /*
         * Individual Subsystem Action (not needed, just here)
         */
        NamedCommands.registerCommand("automatedShoot", Commands.runOnce(() -> {
             getSuperstructure().setWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC);
        }));

        NamedCommands.registerCommand("intake", Commands.runOnce(() -> {
             getSuperstructure().setWantedIntakeState(Superstructure.WantedIntakeState.INTAKING);
        }));

        NamedCommands.registerCommand("passiveFeeding", Commands.runOnce(() -> {
             getSuperstructure().setWantedFeederState(Superstructure.WantedFeederState.SLOW_FEEDING);
        }));

        NamedCommands.registerCommand("activeFeeding", Commands.runOnce(() -> {
             getSuperstructure().setWantedFeederState(Superstructure.WantedFeederState.FAST_FEEDING);
        }));

        NamedCommands.registerCommand("openGatekeeper", Commands.runOnce(() -> {
             getSuperstructure().setWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN);
        }));

        NamedCommands.registerCommand("closeGatekeeper", Commands.runOnce(() -> {
             getSuperstructure().setWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSED);
        }));


        /*
         * Combined subsystem Actions (will be used)
         */
        NamedCommands.registerCommand("snowblowing", Commands.runOnce(() -> { //Intake and shoot balls at the same time
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SNOWBLOWER);
        }));
        NamedCommands.registerCommand("storageShooting", Commands.runOnce(() -> { //Shoots the balls solely in the Feeder
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.STORAGE_SHOOTER);
        }));
        NamedCommands.registerCommand("storageIntake", Commands.runOnce(() -> { //Intake balls into the Feeder without shooting them
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.STORAGE_INTAKE);
        }));
        NamedCommands.registerCommand("l1Climbing", Commands.runOnce(() -> {
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.L1_CLIMB);
        }));
        NamedCommands.registerCommand("idling", Commands.runOnce(() -> {
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.IDLE);
        }));
    }
}
