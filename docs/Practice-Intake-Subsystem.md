# Practice Intake Subsystem

Steps to create a simple Intake subsystem to gain practice working with subsystems. This is in no means an exhaustive example. Rather it has the high level steps. Refer to the ITurrent set of classes and how they are integrated for a complete example

## Create Intake folder under Subsystems
## Create IIntake

* Extend from ITestableSubSystem and SubsystemDataProcessor.IODataRefresher
* Add NAME for yaml file
* Create IntakeIOInputs class with data
* Add refreshData method
* Add update method
* Add set methods

## Create IntakeCTRE implementation

* Extend from IIntake
* Define motors for intake by reading from yaml
* Add data points we will read from robot as StatusSignal types
* Init the data points
* Override update method to copy data into IntakeIOInputs from StatusSignals
* Override refreshData to refresh StatusSignals
* Override set methods

## Create IntakeSubsystem

* Extend from SubsystemBase
* Add a private IntakeIOInputs variable
* Add a private ITurret variable
* Add constructor to take in ITurret and store it
* Create the DataProcessor and have it call the update method in Intake with the IntakeIOInputs as input
* Add a IntakeState enum (INTAKE, OUTTAKE, IDLE)
* Add a SystemState enum (INTAKE, OUTTAKE, IDLE)
* Create a wantedState of type IntakeState
* Create a systemState of type SystemState
* Add a handleStateTransition to handle the switch from wanted to system
* Add an applyStates to look at the current system state and apply power to the intake using the set methods.
* Add a setWantedState to set the IntakeState
* Add a periodic to handle the state transition and apply the states

## Updates to BaseSuperstructure

* Add in a INTAKE\_IDLE, INTAKE\_IN, INTAKE\_OUT to both WantedSuperState and CurrentSuperState
* Add the three states to the TELEOP section of the switch statement in applyStates

## Updates to Superstructure

* Add a IntakeSubsystem private variable
* Update the constructor set the IntakeSubsystem
* Update the handleStateTransitions to set the CurrentSuperState from the WantedSuperState
* Update applyStates to set the intake wanted states to match the CurrentSuperState

## Updates to BaseRobotController

* Add IIntake private variable
* Initialize intake similar to how the turret is created

## Updates to RobotContainer

* Add IntakeSubsystem private variable
* Create the IntakeSubsystem
* Create Superstructure add in Intake
* Configure trigger bindings to intake, outtake, and idle. Left will intake, right will outtake, neither will idle.

## Updates to Yaml file

* Add an intake subsystem with the necessary devices
