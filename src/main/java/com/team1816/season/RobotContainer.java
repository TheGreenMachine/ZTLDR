package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.BaseRobotContainer;
import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Indexer;
import com.team1816.season.subsystems.Superstructure;

public class RobotContainer extends BaseRobotContainer {
    public RobotContainer() {
        NamedCommands.registerCommand("InTheZone", new InTheZoneCommand());
        // call the base to initialize library objects
        // i.e. subsystems that always exist like the drivetrain and path planner
        initializeLibSubSystems();

        Singleton.CreateSubSystem(Indexer.class);

        superstructure = new Superstructure(swerve);

        initializeAutonomous();

        configureBindings();
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    private void configureBindings() {
        controller.a().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT));
        controller.b().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.CLIMBING));
    }
}
