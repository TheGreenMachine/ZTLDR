# Motion-Based Subsystems

This document fully outlines the creation and configuration of a subsystem based around motors. Check the "YAML Configuration" section for specifics on configuring a subsystem.

## Coding a Subsystem

### Subsystem Definition & Initialization

Navigate to `season/subsystems`.

Name it something, and have it implement the `ITestableSubsystem` interface. Put in a `public static final String` named `NAME` at the top of your class defining the name of the subsystem. This name will be used in the YAML.

```java
public class Elevator implements ITestableSubsystem {
    public static final String NAME = "elevator";
}
```

We will also have to initialize the subsystem somewhere in the code. Navigate to `RobotContainer`, and call `Singleton.CreateSubsystem(...)` in its constructor to do so. It will be immediately obvious where in the constructor to do so, since there will be many other calls to this function nearby.

```java
public RobotContainer() {
    // ... bunch of junk

    Singleton.CreateSubsystem(Elevator.class);

    // ... bunch of other junk
}
```


### Device Control

This section is about retrieving and controlling moving parts on the robot.

First, we will retrieve the motor using `factory.getDevice(...)` and store it at the top of the class.
```java
// IMPORTANT: statically import "factory" from Singleton!
import static com.team1816.lib.Singleton.factory;

...

private final IMotor elevatorMotor = (IMotor) factory.getDevice(NAME, "elevatormotor");
private final IMotor followerMotor = (IMotor) factory.getDevice(NAME, "followermotor");
// TODO: include example for non-motor devices
```

We use things called `ControlRequest`s which control motors in certain ways. Begin by creating `ControlRequest` fields at the top of the class. Here are a few examples of `ControlRequest`s:

```java
private VelocityVoltage velReq = new VelocityVoltage(0);
private PositionVoltage posReq = new PositionVoltage(0);
private DutyCycleOut    dcoReq = new DutyCycleOut(0);
// ...and many more. Navigate to com.ctre.phoenix6.controls.ControlRequest and look for inheritors for a full list in your IDE.
```

Conventionally, we define an `applyState()` method (you'll find out why we do things this way later) from which you dispatch `ControlRequest`s to the motor.

```java
private VelocityVoltage elevatorVelReq = new VelocityVoltage(0);

...

public void applyState() {
    elevatorMotor.setControl(elevatorVelReq.withVelocity(1));
}
```

`applyState()` is a method that we'll call within `periodic()`, which is inherited from `ITestableSubsystem`. This will automatically be called every loop. Call `applyState()` within `periodic()`.

```java
@Override
public void periodic() {
    applyState();
}
```

*Every* subsystem will have a `periodic` method.

### State management

The subsystem is now able to control the robot, but there is no way to control the subsystem itself. A subsystem is useless if it is unable to be interacted with.

So how do we interact with a subsystem? We use something called **state management**, which just allows an outside user to control the state of a subsystem. For example, a turret may have these states:
- Idle
- Rotated left
- Rotated right
- Rotated backwards
- Rotated forwards
- Etc.

And an outside user sets the desired state of a subsystem to one of these states. The subsystem must operate according to the current desired state.

So, let's start with the ability to set these states. We will want two things: an enum, which defines the possible states of a subsystem, and a setter from which people can control the subsystem.

Within our elevator subsystem, we will define an enum like so:

```java
public enum ELEVATOR_STATE {
    STOWED,
    LEVEL_1,
    LEVEL_2,
    IDLE
}
```

We will also define a field and setter for the desired state of the subsystem:

```java
private ELEVATOR_STATE wantedState = ELEVATOR_STATE.IDLE;

public void setWantedState(ELEVATOR_STATE state) {
    this.wantedState = wantedState;
}
```

Now, we want to be able to set wanted states from outside the class. To do this, we define a simple field and setter for the subsystem.

```java
private ELEVATOR_STATE wantedState;

...

public void setWantedState(ELEVATOR_STATE wantedState) {
    this.wantedState = wantedState;
}
```

The subsystem observes this `wantedState` field in order to control its motors. This is where `applyState()` comes in: it reads the `wantedState` every loop, and does things based on that value.

```java
private void applyState() {
    switch (wantedState) {
        case STOWED  -> elevatorMotor.setControl(elevatorPosReq.withPosition(0));
        case LEVEL_1 -> elevatorMotor.setControl(elevatorPosReq.withPosition(5));
        case LEVEL_2 -> elevatorMotor.setControl(elevatorPosReq.withPosition(10));
        case IDLE    -> elevatorMotor.setControl(elevatorPosReq.withPosition(0));
    }
}
```

### Reading data from devices

Sometimes, you might want to read data from a device, whether it be sensor input or the speed of a motor.

First define fields that are relevant to what you are reading. In this example, we will retrieve the position of the elevator motor.

```java
private double curPosition;
```

Next, create a `readFromHardware()` method. From here, you will set these fields by directly reading from these devices. Check your IDE to see what data you are able to read from a device.

```java
public void readFromHardware() {
    curPosition = elevatorMotor.getMotorPosition();
}
```

Finally, call this method in `periodic()`. Call this before `applyState()`, since you want the most recently updated values.

```java
@Override
public void periodic() {
    readFromHardware();
    applyState();
}
```

### Using YAML data

Often, we want fields within the subsystem to be configurable via the YAML instead of hardcoding it.

Define any constants under the `constants` section of your YAML. Constants will always be `double`s.

```yaml
subsystems:
    elevator:
        devices:
          ...
        pidConfig:
          ...
        constants:
            level1_position: 5
            level2_position: 10
            stowed_position: 0
```

To retrieve these values, use `factory.getConstant([name of subsystem], [name of constant], [default value])`. Store this value at the top of the subsystem.

If you want to display a warning when the constant is not defined in the YAML, use `factory.getConstant([name of subsystem], [name of constant], [default value], [show warning])`. Making the default value `0` will also display a warning if the configuration does not define the value.

```java
private final double level1Position = factory.getConstant(NAME, "level1_position", 0);       // one way to display a warning
private final double level2Position = factory.getConstant(NAME, "level2_position", 3, true); // another way to display a warning
private final double stowedPosition = factory.getConstant(NAME, "stowed_position", 3);       // does not display a warning
```
