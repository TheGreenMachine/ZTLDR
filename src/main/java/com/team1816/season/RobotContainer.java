package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.BaseRobotContainer;
import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Indexer;
import com.team1816.season.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.Commands;

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

    public void teleopInit() {
        superstructure.setWantedSuperState(Superstructure.WantedSuperState.DEFAULT);
        superstructure.teleopInit();
    }

    private void configureBindings() {
        controller.a().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT));
        controller.x().onTrue(
            Commands.runOnce(() -> superstructure.setClimbSide(Superstructure.ClimbSide.LEFT))
                .andThen(superstructure.setStateCommand(Superstructure.WantedSuperState.CLIMB)
        ));
        controller.y().onTrue(Commands.runOnce(() -> superstructure.setClimbSide(Superstructure.ClimbSide.RIGHT)));
        controller.povUp().onTrue(Commands.runOnce(() -> superstructure.setShooterManualDistancePreset(Superstructure.ShooterManualDistancePreset.DISTANCE_ONE)));
        controller.povRight().onTrue(Commands.runOnce(() -> superstructure.setShooterManualDistancePreset(Superstructure.ShooterManualDistancePreset.DISTANCE_TWO)));
        controller.povDown().onTrue(Commands.runOnce(() -> superstructure.setShooterManualDistancePreset(Superstructure.ShooterManualDistancePreset.DISTANCE_THREE)));
        controller.povLeft().onTrue(Commands.runOnce(() -> superstructure.setShooterManualDistancePreset(Superstructure.ShooterManualDistancePreset.DISTANCE_FOUR)));
        controller.rightBumper().whileTrue(Commands.runOnce(() -> superstructure.setShootingEnabled(true)));
        controller.rightBumper().whileFalse(Commands.runOnce(() -> superstructure.setShootingEnabled(false)));
        controller.rightBumper().whileTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.INTAKING)));
        controller.rightBumper().whileTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.OUTTAKING)));
        controller.rightBumper().whileTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.IDLING)));
        controller.rightBumper().whileTrue(Commands.runOnce(() -> superstructure.setWantedIndexerState(Superstructure.WantedIndexerState.INDEXING)));
        controller.rightBumper().whileTrue(Commands.runOnce(() -> superstructure.setWantedIndexerState(Superstructure.WantedIndexerState.OUTDEXING)));
        controller.rightBumper().whileTrue(Commands.runOnce(() -> superstructure.setWantedIndexerState(Superstructure.WantedIndexerState.IDLING)));
    }
}
