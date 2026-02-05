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
//        driverController.povDown().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT));
//        driverController.povLeft().onTrue(
//            Commands.runOnce(() -> superstructure.setClimbSide(Superstructure.ClimbSide.LEFT))
//                .andThen(superstructure.setStateCommand(Superstructure.WantedSuperState.CLIMB)
//        ));
//        driverController.povRight().onTrue(
//            Commands.runOnce(() -> superstructure.setClimbSide(Superstructure.ClimbSide.RIGHT))
//                .andThen(superstructure.setStateCommand(Superstructure.WantedSuperState.CLIMB)
//        ));

        driverController.rightTrigger().onTrue(Commands.runOnce(() -> superstructure.setWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN)));
        driverController.rightTrigger().onFalse(Commands.runOnce(() -> superstructure.setWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSED)));

        driverController.x().onTrue(Commands.runOnce(() -> superstructure.setWantedShooterState(Superstructure.WantedShooterState.DISTANCE_ONE)));
        driverController.y().onTrue(Commands.runOnce(() -> superstructure.setWantedShooterState(Superstructure.WantedShooterState.DISTANCE_TWO)));
        driverController.b().onTrue(Commands.runOnce(() -> superstructure.setWantedShooterState(Superstructure.WantedShooterState.DISTANCE_THREE)));
        driverController.a().onTrue(Commands.runOnce(() -> superstructure.setWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC)));


        operatorController.y().onTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.INTAKING)));
        operatorController.a().onTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.OUTTAKING)));
        operatorController.x().onTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.IDLING)));
        operatorController.povDown().onTrue(Commands.runOnce(() -> superstructure.setIndexerControlState(Superstructure.IndexerControlState.OVERRIDING)));
        operatorController.povDown().onFalse(Commands.runOnce(() -> superstructure.setIndexerControlState(Superstructure.IndexerControlState.DEFAULTING)));
    }
}
