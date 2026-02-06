# Adding Commands and States to the Superstructure

This guide walks through the complete process of adding new behaviors to the robot's Superstructure (also known as the Coordinator). The Superstructure is responsible for coordinating all subsystems and handling the state machine logic that controls robot behavior.

## Overview: How the Superstructure Works

The Superstructure uses a **state machine pattern** to coordinate robot behaviors. Here's the flow:

```
Controller Button Press → Command → Superstructure State → Subsystem State → Motor Action
```

### Key Concepts

1. **SuperState Enums** - Define the possible states for different robot mechanisms
2. **State Setter Methods** - Private methods that modify the current state
3. **Command Factory Methods** - Public methods that return `Command` objects to change states
4. **`applyStates()` Method** - Translates superstructure states to subsystem states
5. **Button Bindings** - Connect controller inputs to commands in `RobotContainer`

---

## Step-by-Step Guide

### Step 1: Define Your New States

First, decide what new states you need. Open `Superstructure.java` and add your states to an existing enum OR create a new enum.

**Option A: Add to an Existing Enum**

For example, to add a new shooter state:

```java
public enum ShooterSuperState {
    SHOOTER_AUTO,
    SHOOTER_RANGE1,
    SHOOTER_RANGE2,
    SHOOTER_RANGE3,
    SHOOTER_IDLE,
    SHOOTER_AMP     // ← NEW STATE
}
```

**Option B: Create a New SuperState Enum**

If you're adding a completely new mechanism (like a climber), create a new enum:

```java
public enum ClimberSuperState {
    CLIMBER_REST,
    CLIMBER_L1,
    CLIMBER_L2,
    CLIMBER_L3,
    CLIMBER_RETRACT
}
```

Then add a field to track the current state:

```java
protected ClimberSuperState climberSuperState = ClimberSuperState.CLIMBER_REST;
```

---

### Step 2: Create a Private State Setter Method

Add a private method that sets your new state. This method can include validation logic or toggle behavior if needed.

**Simple Setter:**

```java
private void setClimberSuperState(ClimberSuperState climberSuperState) {
    this.climberSuperState = climberSuperState;
}
```

**Setter with Toggle Logic (like the Intake example):**

```java
private void setClimberSuperState(ClimberSuperState climberSuperState) {
    var currentClimberState = climber.getCurrentState();
    
    // Toggle off if we're already in this state
    if (climberSuperState == ClimberSuperState.CLIMBER_L1 && 
        currentClimberState == Climber.CLIMBER_STATE.L1) {
        climberSuperState = ClimberSuperState.CLIMBER_REST;
    }
    
    this.climberSuperState = climberSuperState;
}
```

---

### Step 3: Create a Public Command Factory Method

This method wraps your state setter in a WPILib `Command` so it can be triggered by button bindings.

```java
public Command setClimberCommand(ClimberSuperState climberSuperState) {
    return new InstantCommand(() -> setClimberSuperState(climberSuperState));
}
```

**Note:** These methods return `Command` objects, which is what the WPILib command-based framework expects for button bindings.

---

### Step 4: Update the `applyStates()` Method

The `applyStates()` method runs every robot loop cycle. It reads the current superstructure states and tells each subsystem what to do.

Add a switch statement for your new state:

```java
protected void applyStates() {
    // ... existing code for other states ...

    // Add your new state handling
    switch (climberSuperState) {
        case CLIMBER_L1:
            climber.setWantedState(Climber.CLIMBER_STATE.L1);
            break;
        case CLIMBER_L2:
            climber.setWantedState(Climber.CLIMBER_STATE.L2);
            break;
        case CLIMBER_L3:
            climber.setWantedState(Climber.CLIMBER_STATE.L3);
            break;
        case CLIMBER_RETRACT:
            climber.setWantedState(Climber.CLIMBER_STATE.REST);
            break;
        case CLIMBER_REST:
        default:
            climber.setWantedState(Climber.CLIMBER_STATE.REST);
            break;
    }
}
```

---

### Step 5: Add the Subsystem Reference (if needed)

If your new behavior uses a subsystem not already in the Superstructure, you need to add it.

**In Superstructure.java:**

1. Add the field:
```java
private final Climber climber;
```

2. Initialize it in the constructor:
```java
public Superstructure(Swerve swerve) {
    this.swerve = swerve;
    this.intake = Singleton.get(Intake.class);
    this.shooter = Singleton.get(Shooter.class);
    this.indexer = Singleton.get(Indexer.class);
    this.climber = Singleton.get(Climber.class);  // ← ADD THIS
}
```

---

### Step 6: Configure Button Bindings in RobotContainer

Now connect your commands to controller buttons. Open `RobotContainer.java` and add bindings in the `configureBindings()` method.

#### Available Controller Inputs (Xbox Controller):

| Input | Code |
|-------|------|
| A Button | `controller.a()` |
| B Button | `controller.b()` |
| X Button | `controller.x()` |
| Y Button | `controller.y()` |
| Left Bumper | `controller.leftBumper()` |
| Right Bumper | `controller.rightBumper()` |
| Left Trigger | `controller.leftTrigger()` |
| Right Trigger | `controller.rightTrigger()` |
| D-Pad Up | `controller.povUp()` |
| D-Pad Down | `controller.povDown()` |
| D-Pad Left | `controller.povLeft()` |
| D-Pad Right | `controller.povRight()` |
| Start Button | `controller.start()` |
| Back Button | `controller.back()` |

#### Binding Examples:

**Simple Press (onTrue) - triggers once when button is pressed:**
```java
controller.start().onTrue(superstructure.setClimberCommand(Superstructure.ClimberSuperState.CLIMBER_L1));
```

**Press and Release (onTrue/onFalse) - different actions for press and release:**
```java
controller.rightTrigger()
    .onTrue(superstructure.setShooterGatekeeperCommand(Superstructure.ShooterGatekeeperSuperState.ON))
    .onFalse(superstructure.setShooterGatekeeperCommand(Superstructure.ShooterGatekeeperSuperState.OFF));
```

**While Held (whileTrue) - runs continuously while button is held:**
```java
controller.leftBumper().whileTrue(superstructure.setClimberCommand(Superstructure.ClimberSuperState.CLIMBER_L1));
```

---

## Complete Example: Adding Climber Control to Superstructure

Here's a complete example putting it all together:

### 1. In `Superstructure.java`:

```java
// Add the enum (near the top with other enums)
public enum ClimberSuperState {
    CLIMBER_REST,
    CLIMBER_L1,
    CLIMBER_L2,
    CLIMBER_L3
}

// Add the state field
protected ClimberSuperState climberSuperState = ClimberSuperState.CLIMBER_REST;

// Add the subsystem field
private final Climber climber;

// Update constructor
public Superstructure(Swerve swerve) {
    this.swerve = swerve;
    this.intake = Singleton.get(Intake.class);
    this.shooter = Singleton.get(Shooter.class);
    this.indexer = Singleton.get(Indexer.class);
    this.climber = Singleton.get(Climber.class);
}

// Add private setter
private void setClimberSuperState(ClimberSuperState climberSuperState) {
    this.climberSuperState = climberSuperState;
}

// Add public command factory
public Command setClimberCommand(ClimberSuperState climberSuperState) {
    return new InstantCommand(() -> setClimberSuperState(climberSuperState));
}

// In applyStates(), add:
switch (climberSuperState) {
    case CLIMBER_L1 -> climber.setWantedState(Climber.CLIMBER_STATE.L1);
    case CLIMBER_L2 -> climber.setWantedState(Climber.CLIMBER_STATE.L2);
    case CLIMBER_L3 -> climber.setWantedState(Climber.CLIMBER_STATE.L3);
    case CLIMBER_REST -> climber.setWantedState(Climber.CLIMBER_STATE.REST);
}
```

### 2. In `RobotContainer.java`:

```java
private void configureBindings() {
    // ... existing bindings ...
    
    // Climber controls on D-pad
    controller.povUp().onTrue(superstructure.setClimberCommand(Superstructure.ClimberSuperState.CLIMBER_L1));
    controller.povRight().onTrue(superstructure.setClimberCommand(Superstructure.ClimberSuperState.CLIMBER_L2));
    controller.povDown().onTrue(superstructure.setClimberCommand(Superstructure.ClimberSuperState.CLIMBER_L3));
    controller.povLeft().onTrue(superstructure.setClimberCommand(Superstructure.ClimberSuperState.CLIMBER_REST));
}
```

---

## Coordinating Multiple Subsystems

One of the Superstructure's key roles is coordinating multiple subsystems. Look at how the indexer is handled in the existing code:

```java
// indexer should only be on if we are intaking or shooting
if (shooterGatekeeperSuperState == ShooterGatekeeperSuperState.ON ||
    intakeSuperState == IntakeSuperState.INTAKE_IN) {
    indexer.setWantedState(Indexer.INDEXER_STATE.ON);
} else {
    indexer.setWantedState(Indexer.INDEXER_STATE.OFF);
}
```

This pattern shows how you can:
- Make one subsystem's behavior depend on another's state
- Implement safety interlocks (e.g., don't shoot while climbing)
- Create complex behaviors from simple state combinations

---

## Checklist for Adding New Behavior

- [ ] Define the new state(s) in an enum (new or existing)
- [ ] Add a field to track the state (if new enum)
- [ ] Add the subsystem reference (if new subsystem)
- [ ] Create a private setter method
- [ ] Create a public command factory method
- [ ] Update `applyStates()` to handle the new state
- [ ] Add button bindings in `RobotContainer.configureBindings()`
- [ ] Test in simulation first!

---

## Debugging Tips

1. **Use SmartDashboard** - Add logging to see state changes:
   ```java
   SmartDashboard.putString("Climber State", climberSuperState.toString());
   ```

2. **Check periodic() is running** - The `applyStates()` method is called from `periodic()` which must be running

3. **Verify button bindings** - Make sure your controller is on port 0 (or the correct port)

4. **Test states individually** - Use the simulator to test each state transition before testing combinations

---

## Files Modified Summary

| File | What to Add |
|------|-------------|
| `Superstructure.java` | Enum, state field, setter, command factory, applyStates logic |
| `RobotContainer.java` | Button bindings in `configureBindings()` |
| Subsystem file (if new) | Create with `ITestableSubsystem` pattern |
| YAML file (if new subsystem) | Device configuration |

---

## Reference: Existing State Examples

### Intake States (with toggle logic):
- `INTAKE_IN` - Run intake wheels inward
- `INTAKE_OUT` - Run intake wheels outward  
- `INTAKE_DOWN` - Lower the intake arm
- `INTAKE_UP` - Raise the intake arm

### Shooter States:
- `SHOOTER_AUTO` - Auto-calculate shooting parameters
- `SHOOTER_RANGE1/2/3` - Preset shooting configurations
- `SHOOTER_IDLE` - Shooter off

### Gatekeeper States:
- `ON` - Feed balls to shooter
- `OFF` - Stop feeding

### Drive States:
- `TELEOP_DRIVE` - Normal driving
- `TELEOP_IDLE` - Stopped

---

*For questions, ask a mentor or refer to the existing subsystem implementations as examples!*
