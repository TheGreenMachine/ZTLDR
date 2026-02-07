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

        NamedCommands.registerCommand("indexing", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedIndexerState(Superstructure.WantedIndexerState.INDEXING); //TODO: MAY NEED TO CHANGE
        }));

        NamedCommands.registerCommand("outdexing", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedIndexerState(Superstructure.WantedIndexerState.OUTDEXING); //TODO: MAY NEED TO CHANGE
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

        NamedCommands.registerCommand("intaking", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedIntakeState(Superstructure.WantedIntakeState.INTAKING);
            robotContainer.getSuperstructure().setWantedIndexerState(Superstructure.WantedIndexerState.INDEXING);
        }));

        NamedCommands.registerCommand("automatedShooting", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedIndexerState(Superstructure.WantedIndexerState.OUTDEXING);
            robotContainer.getSuperstructure().setWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN);
            robotContainer.getSuperstructure().setWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC);
        }));

        NamedCommands.registerCommand("automatedShooting", Commands.runOnce(() -> {
            robotContainer.getSuperstructure().setWantedIndexerState(Superstructure.WantedIndexerState.OUTDEXING);
            robotContainer.getSuperstructure().setWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN);
            robotContainer.getSuperstructure().setWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC);
        }));

        NamedCommands.registerCommand("snowblowerShooting", Commands.runOnce(() -> {
            //What would be in here? (indexer mainly)
        }));





    }
}
