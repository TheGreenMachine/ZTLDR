package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.BaseRobotContainer;
import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Indexer;
import com.team1816.season.subsystems.Superstructure;

public class RobotContainer extends BaseRobotContainer<Superstructure> {
    public RobotContainer() {
        NamedCommands.registerCommand("InTheZone", new InTheZoneCommand());
        // call the base to initialize library objects
        // i.e. subsystems that always exist like the drivetrain and path planner
        initializeLibSubSystems();

        Singleton.CreateSubSystem(Indexer.class);

        initializeAutonomous();

        configureBindings();
    }

    @Override
    protected Superstructure createSuperstructure() {
        return new Superstructure(swerve, vision);
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    private void configureBindings() {
        controller.leftTrigger().and(controller.rightTrigger().negate()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_IN))
            .or(controller.rightTrigger().and(controller.leftTrigger().negate()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_OUT)))
            .or(controller.leftTrigger().and(controller.rightTrigger()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_IDLE)))
            .or(controller.leftTrigger().negate().and(controller.rightTrigger().negate()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_IDLE)));
//        controller.y().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_TO_0))
//            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_IDLE));
//        controller.a().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_TO_180))
//            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_IDLE));
    }
}
