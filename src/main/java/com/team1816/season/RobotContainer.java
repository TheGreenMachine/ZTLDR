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

        driverController.povUp().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_1)));
        driverController.povRight().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_2)));
        driverController.povLeft().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_3)));
        driverController.povDown().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_AUTOMATIC_HUB)));

        driverController.b().onTrue(Commands.runOnce(() -> superstructure.toggleIntake()));
        driverController.y().onTrue(Commands.runOnce(() -> superstructure.toggleIntakeDeployment()));
        //Hood down/up TBD, may be an automated movement
        //Agitate button TBD on use
        driverController.x().onTrue(Commands.runOnce(() -> superstructure.toggleAgitate()));


        //add manual shooter speed adjustments

// CLIMBING MODE CONTROLS:

//        driverController.y().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.CLIMBER_CLIMB_L3)));
//        driverController.a().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.CLIMBER_CLIMB_L1)));
//        driverController.b().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.CLIMBER_CLIMB_DOWN_L1)));

        //add manual climber up/down adjustments and the extend/retract adjustments
    }

    public final void registerCommands() {
        NamedCommands.registerCommand("automatedHubShooting", Commands.runOnce(() -> {
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_AUTOMATIC_HUB);
        }));
        NamedCommands.registerCommand("automatedCorner1Shooting", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_AUTOMATIC_CORNER); //Figure out which corners are which
        }));

        NamedCommands.registerCommand("automatedHubSnowblowing", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SNOWBLOWER_AUTOMATIC_HUB);
        }));
        NamedCommands.registerCommand("automatedCorner1Snowblowing", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SNOWBLOWER_AUTOMATIC_CORNER); //Figure out which corners are which
        }));

        NamedCommands.registerCommand("intaking", Commands.runOnce(() -> {
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.INTAKE);
        }));

        NamedCommands.registerCommand("slowFeeding", Commands.runOnce(() -> {
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.FEEDER_SLOW);
        }));
        NamedCommands.registerCommand("fastFeeding", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.FEEDER_FAST);
        }));

        NamedCommands.registerCommand("openGatekeeper", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.GATEKEEPER_ON);
        }));

        NamedCommands.registerCommand("closeGatekeeper", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.GATEKEEPER_OFF);
        }));
        NamedCommands.registerCommand("l1Climbing", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.CLIMB_L1);
        }));
        NamedCommands.registerCommand("defaulting", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.DEFAULT);
        }));
    }
}
