package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.season.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.Commands;

/**TODO: 1. Refactor indexStates (not quite right)
 *       2. Create Combined Actions in superstructure Actions for here
 *       3. Make sure this is in line with the current conventions of the codebase
 *       4. Make sure it works within pathplanner
 *       5. Generally update subsystem
 *
 */

public class NamedCommandRegistrar {
    private RobotContainer robotContainer;

    public final void RegisterCommands() {
        /**
         * Individual Subsystem Action (not needed, just here)
        */
        NamedCommands.registerCommand("automatedShoot", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC);
        }));

        NamedCommands.registerCommand("intake", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedIntakeState(Superstructure.WantedIntakeState.INTAKING);
        }));

        NamedCommands.registerCommand("passiveFeeding", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedIndexerState(Superstructure.WantedIndexerState.PASSIVE_FEEDING);
        }));

        NamedCommands.registerCommand("activeFeeding", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedIndexerState(Superstructure.WantedIndexerState.ACTIVE_FEEDING);
        }));

        NamedCommands.registerCommand("openGatekeeper", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN);
        }));

        NamedCommands.registerCommand("closeGatekeeper", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSED);
        }));


        /**
         * Combined subsystem Actions (will be used)
         */
        NamedCommands.registerCommand("snowblowing", Commands.runOnce(() -> { //Intake and shoot balls at the same time
            robotContainer.getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SNOWBLOWER);
        }));
        NamedCommands.registerCommand("storageShooting", Commands.runOnce(() -> { //Shoots the balls solely in the indexer(Feeder)
            robotContainer.getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.STORAGE_SHOOTER);
        }));
        NamedCommands.registerCommand("storageIntake", Commands.runOnce(() -> { //Intake balls into the indexer(Feeder) without shooting them
            robotContainer.getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.STORAGE_INTAKE);
        }));
        NamedCommands.registerCommand("l1Climbing", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.L1_CLIMB);
        }));
        NamedCommands.registerCommand("idling", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.IDLE);
        }));







    }
}
