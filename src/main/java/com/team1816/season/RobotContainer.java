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
//        driverController.povDown().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT));
//        driverController.povLeft().onTrue(
//            Commands.runOnce(() -> superstructure.setClimbSide(Superstructure.ClimbSide.LEFT))
//                .andThen(superstructure.setStateCommand(Superstructure.WantedSuperState.CLIMB)
//        ));
//        driverController.povRight().onTrue(
//            Commands.runOnce(() -> superstructure.setClimbSide(Superstructure.ClimbSide.RIGHT))
//                .andThen(superstructure.setStateCommand(Superstructure.WantedSuperState.CLIMB)
//        ));

        driverController.rightTrigger().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.GATEKEEPER_ON)));
        driverController.rightTrigger().onFalse(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.GATEKEEPER_OFF)));

        driverController.x().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_1)));
        driverController.y().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_2)));
        driverController.b().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_3)));
        driverController.a().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_AUTOMATIC_HUB)));
        //Would want the auto-targeting to be a toggle between hub & cornerX(Decided automatically by default)

        driverController.rightBumper().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.INTAKE)));
        driverController.povUp().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.INTAKE_LIFT)));
        driverController.povDown().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.INTAKE_DROP)));

        //operatorController.x().onTrue(Commands.runOnce(() -> superstructure.setWantedIntakeState(Superstructure.WantedIntakeState.UP)));
        operatorController.povDown().onTrue(Commands.runOnce(() -> superstructure.setFeederControlState(Superstructure.FeederControlState.OVERRIDING))); //Still not sure what this is doing
        operatorController.povDown().onFalse(Commands.runOnce(() -> superstructure.setFeederControlState(Superstructure.FeederControlState.DEFAULTING)));

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
