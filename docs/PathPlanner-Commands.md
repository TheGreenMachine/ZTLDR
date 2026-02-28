# PathPlanner Commands

- For documentation on creating _autos_ using PathPlanner, please look at the [official PathPlanner docs](https://pathplanner.dev/pathplanner-gui.html).
- For documentation on calling the commands we register in the code from PathPlanner GUI, please look at the [official PathPlanner auto docs](https://pathplanner.dev/gui-editing-paths-and-autos.html#autos).

PathPlanner communicates with the two robot in two ways: utilizing the drivetrain and **commands**. Every season, the commands being used are different, and it is essential to implement commands so PathPlanner can make subsystems do stuff.

PathPlanner uses its own interface called `NamedCommands`. In the code, you must call `NamedCommands.registerCommand()` to map a command string (i.e. "turretSpinTo180") to an actual, executable WPILib command.

Conventially, we like to register all of our `NamedCommands` in one place. There should be a method called `registerCommands()` inside your season's `RobotContainer`. Within it, you should register all the commands.

```java
private void registerCommands() {
    /*
     * Individual Subsystem Action (not needed, just here)
     */
    NamedCommands.registerCommand("automatedShoot", Commands.runOnce(() -> {
        getSuperstructure().setWantedShooterState(Superstructure.WantedShooterState.AUTOMATIC);
    }));

    NamedCommands.registerCommand("intake", Commands.runOnce(() -> {
        getSuperstructure().setWantedIntakeState(Superstructure.WantedIntakeState.INTAKING);
    }));

        ...


    /*
     * Combined subsystem Actions
     */
    NamedCommands.registerCommand("snowblowing", Commands.runOnce(() -> { //Intake and shoot balls at the same time
        getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.SNOWBLOWER);
    }));
    NamedCommands.registerCommand("storageShooting", Commands.runOnce(() -> { //Shoots the balls solely in the Feeder
        getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.STORAGE_SHOOTER);
    }));

        ...

}
```

Here are some baseline rules:
- **Try to only access the Superstructure from these commands.** The Superstructure is the controller of the whole robot. PathPlanner should never have to interface with a single subsystem during an auto.
- **Only use `Commands.runOnce()` or `InstantCommands` with registered commands.** As said before, the Superstructure is the main controller of everything. If you need to run something later or repeatedly, it should be handled in the Superstructure.
