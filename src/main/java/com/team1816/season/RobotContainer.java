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
        registerCommands();
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

        driverController.rightBumper().onTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.INTAKING)));
        driverController.leftBumper().onTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.OUTTAKING)));
        driverController.povUp().onTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.UP)));
        driverController.povDown().onTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.DOWN)));

        //operatorController.x().onTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.UP)));
        operatorController.povDown().onTrue(Commands.runOnce(() -> superstructure.setIndexerControlState(Superstructure.IndexerControlState.OVERRIDING)));
        operatorController.povDown().onFalse(Commands.runOnce(() -> superstructure.setIndexerControlState(Superstructure.IndexerControlState.DEFAULTING)));

        // controller.leftTrigger().and(controller.rightTrigger().negate()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_IN))
        //     .or(controller.rightTrigger().and(controller.leftTrigger().negate()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_OUT)))
        //     .or(controller.leftTrigger().and(controller.rightTrigger()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_UP)))
        //     .or(controller.leftTrigger().negate().and(controller.rightTrigger().negate()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_UP)));
//        controller.y().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_TO_0))
//            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_IDLE));
//        controller.a().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_TO_180))
//            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_IDLE));
        // controller.povUp().whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_UP));
        // controller.povDown().whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_DOWN));
        // controller.leftBumper().whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_OUT));
        // controller.rightBumper().whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_IN));
    }
    public final void registerCommands() {
        /**
         * Individual Subsystem Action (not needed, just here)
         */
        NamedCommands.registerCommand("automatedShoot", Commands.runOnce(() -> {
             getSuperstructure().setWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC);
        }));

        NamedCommands.registerCommand("intake", Commands.runOnce(() -> {
             getSuperstructure().setWantedIntakeState(Superstructure.WantedIntakeState.INTAKING);
        }));

        NamedCommands.registerCommand("passiveFeeding", Commands.runOnce(() -> {
             getSuperstructure().setWantedIndexerState(Superstructure.WantedIndexerState.PASSIVE_FEEDING);
        }));

        NamedCommands.registerCommand("activeFeeding", Commands.runOnce(() -> {
             getSuperstructure().setWantedIndexerState(Superstructure.WantedIndexerState.ACTIVE_FEEDING);
        }));

        NamedCommands.registerCommand("openGatekeeper", Commands.runOnce(() -> {
             getSuperstructure().setWantedGatekeeperState(Superstructure.WantedGatekeeperState.OPEN);
        }));

        NamedCommands.registerCommand("closeGatekeeper", Commands.runOnce(() -> {
             getSuperstructure().setWantedGatekeeperState(Superstructure.WantedGatekeeperState.CLOSED);
        }));


        /**
         * Combined subsystem Actions (will be used)
         */
        NamedCommands.registerCommand("snowblowing", Commands.runOnce(() -> { //Intake and shoot balls at the same time
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SNOWBLOWER);
        }));
        NamedCommands.registerCommand("storageShooting", Commands.runOnce(() -> { //Shoots the balls solely in the indexer(Feeder)
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.STORAGE_SHOOTER);
        }));
        NamedCommands.registerCommand("storageIntake", Commands.runOnce(() -> { //Intake balls into the indexer(Feeder) without shooting them
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
