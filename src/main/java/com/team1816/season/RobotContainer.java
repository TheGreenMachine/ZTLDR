package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.BaseRobotContainer;
import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Feeder;
import com.team1816.season.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer extends BaseRobotContainer {

    private ControllerMode currentControllerMode;

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

    public enum ControllerMode {
        CLIMBING,
        SNOWBLOWING,
    }

    private void configureBindings() {


        switch (currentControllerMode) {
            case SNOWBLOWING -> {
                driverController.start().onTrue(Commands.runOnce(() -> setCurrentControllerMode(ControllerMode.CLIMBING)));

                driverController.rightTrigger().whileTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.GATEKEEPER_ON)));
                driverController.rightTrigger().onFalse(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.GATEKEEPER_OFF)));

                //one or the other, depends on drive team:
                driverController.povUp().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_1)));
                driverController.povRight().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_2)));
                driverController.povLeft().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_3)));
                driverController.povDown().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_AUTOMATIC_HUB)));

                operatorController.y().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_1)));
                operatorController.x().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_2)));
                operatorController.b().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_DISTANCE_3)));
                operatorController.a().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_AUTOMATIC_HUB)));

                driverController.leftBumper().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.INTAKE_DROP)));
                driverController.rightBumper().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.INTAKE_LIFT)));


                //agitate button TBD
                //add manual shooter speed adjustments
            }
            case CLIMBING -> {
                driverController.back().onTrue(Commands.runOnce(() -> setCurrentControllerMode(ControllerMode.SNOWBLOWING)));

                driverController.y().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.CLIMB_L3)));
                driverController.a().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.CLIMB_L1)));
                driverController.b().onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(Superstructure.WantedSuperState.CLIMB_DOWN_L1)));

                //add manual climber up/down adjustments and the extend/retract adjustments
            }
            //manual mode?
        }

    }

    private void setCurrentControllerMode(ControllerMode wantedControllerMode){
        currentControllerMode = wantedControllerMode;
        configureBindings();
    }

    public final void registerCommands() {
        NamedCommands.registerCommand("automatedHubShooting", Commands.runOnce(() -> {
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_AUTOMATIC_HUB);
        }));
        NamedCommands.registerCommand("automatedCorner1Shooting", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SNOWBLOWER_AUTOMATIC_CORNER); //Figure out which corners are which
        }));

        NamedCommands.registerCommand("automatedHubSnowblowing", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SHOOTER_AUTOMATIC_HUB);
        }));
        NamedCommands.registerCommand("automatedCorner1Snowblowing", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SNOWBLOWER_AUTOMATIC_CORNER); //Figure out which corners are which
        }));

        NamedCommands.registerCommand("intaking", Commands.runOnce(() -> {
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.INTAKE);
        }));

        /*
        NamedCommands.registerCommand("slowFeeding", Commands.runOnce(() -> {
             getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.FEEDER_SLOW);
        }));
        NamedCommands.registerCommand("fastFeeding", Commands.runOnce(() -> {
            getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.FEEDER_FAST);
        }));

         */
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
