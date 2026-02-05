# 🎯 Ballistic Calculator — Student Code Review & Task Guide

> **Note to students:** This guide identifies issues in your current implementation and gives you step-by-step instructions to fix them. You should be able to complete all of these tasks using the concepts from `ballistics-calc.md`, your physics/math knowledge, and the WPILib docs. If you truly get stuck, the answer key is in `lib/util/ballisticCalc` — but try to work through it first!

---

## Table of Contents

1. [Overview: What's Different?](#1-overview)
2. [File-by-File Issues](#2-file-by-file-issues)
   - [ballisticConstants.java](#21-ballisticconstantsjava)
   - [BallisticConstraints.java](#22-ballisticconstraintsjava)
   - [BallisticSolution.java](#23-ballisticsolutionjava)
   - [BallisticCalculator.java](#24-ballisticcalculatorjava)
3. [Missing Features You Need to Add](#3-missing-features)
4. [Task Checklist](#4-task-checklist)

---

<a name="1-overview"></a>
## 1. Overview: What's Different?

Your student implementation has the right *idea* — you've identified that you need constants, constraints, a solution object, and a calculator. Great start! However, there are significant gaps between where you are and a working ballistic calculator:

| Area | Student Version | What It Needs To Be |
|------|----------------|---------------------|
| **Constants/Properties** | `ballisticConstants` — mixes constants with configurable state, getters act as setters | An immutable data class for ball properties (diameter, mass) |
| **Constraints** | `BallisticConstraints` — single "enter angle" with a setter-getter | Full constraint ranges: min/max velocity, min/max pitch, min/max yaw |
| **Solution** | `BallisticSolution` — stores 3 values, no success/failure concept | Full solution with success/failure pattern, velocity vector, flight time, all angles |
| **Calculator** | `BallisticCalculator` — has distance/angle math with bugs, no iterative solver | Full iterative solver using Newton-Raphson, vehicle velocity compensation, constraint validation |
| **Error Handling** | None | Custom exception class, failure reasons in solutions |
| **3D Support** | Mixed 2D/3D, inconsistent | Fully 3D using WPILib `Pose3d`, `Translation3d` |

---

<a name="2-file-by-file-issues"></a>
## 2. File-by-File Issues

---

<a name="21-ballisticconstantsjava"></a>
### 2.1 `ballisticConstants.java`

**Current Problems:**

#### Problem A: Java Naming Convention Violation
Your class name `ballisticConstants` starts with a lowercase letter. In Java, class names should **always** start with an uppercase letter.

> **Task:** Rename the class to follow Java conventions. Think about what this class *actually represents* — is it really "constants"? The answer key uses a name that describes the physical object being modeled.

#### Problem B: The "Getter-Setter" Anti-Pattern 🚨
This is the **biggest conceptual issue** in your code. Look at this pattern you've used:

```java
public double getGravity(double gravity) {
    this.gravity = gravity;
    return gravity;
}
```

This is a method named `get...` that **takes a parameter and overwrites the stored value** with it. This is deeply problematic:
- A "getter" should *retrieve* a value, not *change* it
- A "setter" should *set* a value, not *return* it
- This method does both badly — the stored value gets overwritten every time someone "gets" it

**Think about it this way:** If you ask someone "Hey, what's your phone number?" and they respond "Whatever number you just told me!" — that's what your getters are doing.

> **Task:** Decide which values are truly *constants* (like gravity — is that really going to change?) and which are *properties* of your specific ball/game piece. 
> 
> Constants should be `static final` fields:
> ```java
> private static final double GRAVITY = 9.81; // This never changes!
> ```
> 
> Properties of the ball (diameter, mass) should be set **once** in the constructor and never changed (make them `final`):
> ```java
> private final double diameter;
> private final double mass;
> 
> public BallProperties(double diameter, double mass) {
>     this.diameter = diameter;
>     this.mass = mass;
> }
> 
> public double getDiameter() {  // A REAL getter: no parameter, just returns the value
>     return diameter;
> }
> ```

#### Problem C: "Fun Factors" Are a Good Instinct But Wrong Location
Your `funFactorStagnant` and `funFactorMobile` fields show good engineering thinking — real robots need calibration fudge factors! However:
- These don't belong in ball properties — they're more about your launcher's behavior
- They should be part of a calibration/tuning system, not the physics calculator
- For now, focus on getting the physics *right* first, then add correction factors later

> **Task:** Remove the fun factors from this class for now. Make a note/TODO comment to add a calibration system later. Get the math working first.

#### Problem D: Gravity value inconsistency
You declare `private double gravity = 9.8;` but then your getter overwrites it. Standard gravity is 9.81 m/s². Pick one value, make it a constant, and use it consistently everywhere.

---

<a name="22-ballisticconstraintsjava"></a>
### 2.2 `BallisticConstraints.java`

**Current Problems:**

#### Problem A: Way Too Minimal
Your constraints class only has one field: `enterAngle`. A real launcher has many physical limits:
- **Velocity range:** How slow/fast can you launch? (e.g., 1-30 m/s)
- **Pitch range:** How low/high can you aim the launcher? (e.g., -10° to 80°)  
- **Yaw range:** How far can the turret rotate? (e.g., ±180°)

> **Task:** Add fields for all six constraint boundaries: `minLaunchVelocity`, `maxLaunchVelocity`, `minPitchAngle`, `maxPitchAngle`, `minYawAngle`, `maxYawAngle`.

#### Problem B: Same Getter-Setter Anti-Pattern
Same issue as `ballisticConstants`:
```java
public double getEnterAngle(double enterAngle) {
    this.enterAngle = enterAngle;
    return enterAngle;
}
```

> **Task:** Use a constructor to set all constraints at creation time. Make fields `final`. Provide real getters (no parameters). Consider adding a `static defaults()` factory method so there's an easy way to get reasonable default values.

#### Problem C: No Validation Methods
The constraints class should have methods that *check* if a given value is within bounds:

```java
// Example pattern — you need methods like this:
public boolean isSomethingValid(double value) {
    return value >= minimum && value <= maximum;
}
```

> **Task:** Add `isVelocityValid(double)`, `isPitchValid(double)`, and `isYawValid(double)` methods.

#### Problem D: Degrees vs. Radians
Be very careful about units! Decide on a convention:
- Store angles internally in **radians** (math functions like `Math.sin()` expect radians)
- Accept angles from the user in **degrees** (more human-readable)
- Convert in the constructor using `Math.toRadians()`

> **Task:** Make the constructor accept degrees (for human friendliness) but store and return radians (for math friendliness).

---

<a name="23-ballisticsolutionjava"></a>
### 2.3 `BallisticSolution.java`

**Current Problems:**

#### Problem A: No Success/Failure Concept
Your solution always contains numbers, but what happens when no valid shot exists? (Target is too far, angle impossible, etc.) You need a way to represent "I couldn't find a solution."

> **Task:** Add a `boolean solutionFound` field. Create two static factory methods:
> ```java
> public static BallisticSolution success(/* all the parameters */) { ... }
> public static BallisticSolution failure(String reason) { ... }
> ```
> 
> The `failure` method should return a solution object where `solutionFound = false` and store the reason. This pattern is called the **"Result Object" pattern** — instead of throwing an exception or returning null, you return an object that *tells you what happened*.

#### Problem B: Missing Critical Fields
Your solution has `rotationAngle`, `launchAngle`, and `launchVelocity`. You're missing:
- **Flight time** — how long the ball is in the air
- **Pitch angle** — elevation angle (you have launchAngle but the naming is ambiguous)
- **Yaw angle** — horizontal aiming direction
- **Launch velocity *vector*** — the full 3D velocity, not just the scalar speed
- **Predicted landing position** — where you expect the ball to actually land
- **Landing angle** — the angle at which the ball arrives at the target
- **Failure reason** — a string explaining why no solution was found

> **Task:** Expand the class to include all these fields. Use `Translation3d` for the velocity vector and landing position. Add getters that return degrees for angles (since the fields store radians).

#### Problem C: No `toString()` Method
When debugging, you'll want to print your solution. Add a `toString()` that formats nicely:
```java
@Override
public String toString() {
    if (!solutionFound) {
        return "BallisticSolution[FAILED: " + failureReason + "]";
    }
    return String.format("BallisticSolution[velocity=%.2f m/s, pitch=%.2f°, ...]", ...);
}
```

#### Problem D: Unused Import
You import `Pose2d` but never use it. Clean up your imports.

---

<a name="24-ballisticcalculatorjava"></a>
### 2.4 `BallisticCalculator.java`

This is where the bulk of the issues are. Let's go through them carefully.

#### Problem A: Unused/Unnecessary Imports and Fields
```java
import com.team1816.season.RobotContainer;
import com.team1816.season.RobotState;
```
Your calculator should be a **pure math library** — it shouldn't depend on robot-specific classes. This makes it impossible to test independently and violates separation of concerns.

You also have fields like `MissleTranslation`, `RobotTranslation`, `RobotSpeed`, `hubTranslation` stored as instance state — but a calculator should take inputs and produce outputs, not store intermediate state like this.

> **Task:** Remove all imports and references to `RobotContainer` and `RobotState`. The calculator should accept everything it needs as method parameters.

#### Problem B: The Getter Methods Don't Make Sense
```java
public Translation3d getRobotTranslation(Translation3d missileTranslation) {
    this.MissleTranslation = missileTranslation;
    return MissleTranslation;
}
```

This sets `MissleTranslation` to the *passed-in* value and returns it. It's the same anti-pattern from your constants class, but applied to Translation3d objects. These methods also have confusing names — `getRobotTranslation` takes a *missile* translation?

> **Task:** Remove these methods. Instead, pass all positions directly into your calculation method. A calculator takes inputs and returns outputs — it shouldn't store intermediate state.

#### Problem C: Critical Bug in Distance Calculation 🐛🐛🐛
This appears **three times** in your code:
```java
var distance = Math.sqrt(
    (missileTranslation.getX() - targetStartTranslation.getX()) * 
     missileTranslation.getX() - targetStartTranslation.getX()) + 
    ((missileTranslation.getY() - targetStartTranslation.getY() * 
      missileTranslation.getY() - targetStartTranslation.getY()))
);
```

There are **multiple math errors** here:

1. **Missing parentheses in the squaring operation.** When you want to compute `(a - b)²`, you need to write `(a - b) * (a - b)`. What you've written computes something completely different due to operator precedence.

2. **The parentheses are mismatched** — look carefully at where they open and close. `getY()` values are being multiplied with each other in the wrong grouping.

Here's how to correctly compute the distance between two points in the XY plane:
```java
// The pattern for squaring a difference:
double dx = pointA.getX() - pointB.getX();
double dy = pointA.getY() - pointB.getY();
double distance = Math.sqrt(dx * dx + dy * dy);

// This is the same as: sqrt( (x1-x2)² + (y1-y2)² )
// A very common mistake is writing: (a - b) * a - b
// which Java evaluates as: ((a - b) * a) - b  ← NOT what you want!
// You MUST write: (a - b) * (a - b)
```

> **Task:** Fix the distance calculation everywhere it appears. **Extract it into a helper method** so you only write it once:
> ```java
> private double horizontalDistance(Translation3d from, Translation3d to) {
>     double dx = from.getX() - to.getX();
>     double dy = from.getY() - to.getY();
>     return Math.sqrt(dx * dx + dy * dy);
> }
> ```

#### Problem D: Height Calculation Uses `Math.abs()` Incorrectly
```java
var height = Math.abs(missileTranslation.getZ() - targetStartTranslation.getZ());
```

Using `Math.abs()` loses the **sign** of the height difference. You need to know whether the target is *above* or *below* the launcher — that changes the physics completely! A ball shot upward at a high target behaves very differently from one shot downward.

> **Task:** Remove `Math.abs()`. Use signed height difference: `target.getZ() - launcher.getZ()`. Positive means the target is higher, negative means lower.

#### Problem E: The Launch Angle Formula Is Incorrect
```java
var Theta = Math.toDegrees(Math.atan(((2 * height) / distance)) - Math.tan(enterAngle));
```

Several issues:
1. You're subtracting `Math.tan(enterAngle)` from `Math.atan(...)`. These are different units/scales — you can't subtract a tangent from an arctangent meaningfully.
2. The formula `atan(2h/d)` is a simplified approximation that doesn't account for gravity, velocity, or the landing angle constraint properly.
3. You're converting to degrees, but then using this value in `calculateMissleVelocity` where it's passed into `Math.cos()` and `Math.tan()` — those trig functions expect **radians**, not degrees!

> **Task:** You need to completely rethink the angle calculation. The correct approach is an **iterative solver** (Newton-Raphson method). Here's the concept:
> 
> Instead of a single formula, you:
> 1. Guess a flight time `t`
> 2. Calculate what velocities would be needed for that `t`
> 3. Check if the vertical displacement works out
> 4. If not, refine your guess and try again
> 
> Look at the `ballistics-calc.md` doc, Part 7, for the iteration algorithm. The key insight is:
> 
> ```
> Given a guess for time t:
>   horizontal_velocity = distance / t
>   final_vertical_velocity = -horizontal_velocity * tan(landing_angle)
>   initial_vertical_velocity = final_vertical_velocity + gravity * t
>   predicted_height = initial_vertical_velocity * t - 0.5 * gravity * t²
>   error = predicted_height - actual_height
>   
>   If error is small enough → you found the answer!
>   If not → adjust t using: t_new = t - error / (gravity * t)
> ```

#### Problem F: The Velocity Formula Has Issues
```java
var missleVelocity = ((distance / Math.cos(launchAngle)) * 
    Math.sqrt(Gravity / (2 * (distance * Math.tan(launchAngle)) - height)));
```

This formula is from the "range equation" of projectile motion, which is a closed-form solution that only works under specific conditions (same launch and landing height, no desired landing angle constraint). Since your problem has a *desired landing angle*, you can't use a closed-form formula — you need the iterative approach.

Also, you're passing the result of `calculateLaunchAngle()` (which returns degrees) into `Math.cos()` and `Math.tan()` which expect radians. This will give wildly wrong results.

> **Task:** Replace the velocity formula with the iterative solver. Once you solve for time `t` using the iteration, all the velocities fall out directly:
> ```
> v_horizontal = distance / t
> v_vertical = (final_vertical_v) + gravity * t
> launch_speed = sqrt(v_horizontal² + v_vertical²)
> ```

#### Problem G: `getRotationAngle` Uses `Math.acos` — Fragile
```java
var angle = Math.toDegrees(Math.acos(xDistance / distance));
```

Using `acos` for angle calculation is fragile — it only works for angles 0-180° and loses information about which quadrant the angle is in. The standard approach is `Math.atan2(y, x)` which handles all quadrants correctly and never divides by zero.

> **Task:** Use `Math.atan2()` for yaw calculation:
> ```java
> double yaw = Math.atan2(
>     target.getY() - launcher.getY(),
>     target.getX() - launcher.getX()
> );
> ```

#### Problem H: Vehicle Velocity Compensation Is Incomplete
Your `findACTUALLaunchAngle` method attempts to compensate for robot velocity, which is great! But:
1. It only works in 2D (uses `Translation2d` for velocity)
2. It doesn't decompose the robot velocity into toward-target and lateral components
3. It doesn't account for vertical vehicle velocity

The correct approach (from the md doc, Part 5):
```java
// Project robot velocity onto target direction
double vToward = robotVel.getX() * Math.cos(yawToTarget) + 
                 robotVel.getY() * Math.sin(yawToTarget);

// Perpendicular component (causes lateral drift)
double vLateral = -robotVel.getX() * Math.sin(yawToTarget) + 
                   robotVel.getY() * Math.cos(yawToTarget);

// Vertical
double vVertical = robotVel.getZ();
```

Then subtract these from the required total velocities to get the launch velocities *relative to the robot*.

> **Task:** Rewrite velocity compensation to work in 3D using the projection formulas above.

#### Problem I: `getBallisticSolution` Returns Dummy Data
```java
public BallisticSolution getBallisticSolution(...) {
    return new BallisticSolution(0, 0, 0);
}
```

This is just a placeholder. It needs to actually call your solver.

> **Task:** This should be your main `calculate()` method. It should:
> 1. Take a launcher Pose3d, vehicle velocity (Translation3d), target position (Translation3d), and desired landing angle (double, in degrees)
> 2. Compute the relative position to target
> 3. Compute horizontal distance and vertical displacement
> 4. Compute yaw to target
> 5. Decompose vehicle velocity
> 6. Call the iterative solver
> 7. Validate against constraints
> 8. Return a BallisticSolution (success or failure)

#### Problem J: No Main Calculation Method Signature Uses `Pose3d`
The answer key's main method takes a `Pose3d` for the launcher (which includes both position AND orientation). Your code takes bare `Translation3d` values. Using `Pose3d` is more correct because in the real robot, the launcher's orientation matters.

> **Task:** Change the main calculation method signature to accept `Pose3d launcherPose` instead of separate translation parameters.

---

<a name="3-missing-features"></a>
## 3. Missing Features You Need to Add

### 3.1 Custom Exception Class
Create a `BallisticException` class that extends `RuntimeException`. This is simple:
```java
public class BallisticException extends RuntimeException {
    public BallisticException(String message) {
        super(message);
    }
    // Also add a constructor that takes message + cause (a Throwable)
}
```

### 3.2 Newton-Raphson Iterative Solver
This is the **heart** of the calculator that you're missing entirely. The concept:

1. You need to define constants: `EPSILON = 1e-6` (convergence threshold), `MAX_ITERATIONS = 100`
2. Start with an initial guess for flight time: `t = sqrt(2 * distance / gravity)` (time for a 45° shot)
3. Loop up to MAX_ITERATIONS:
   - From `t`, compute `vh = distance / t`
   - From landing angle, compute final vertical velocity: `vz_final = -vh * tan(landingAngle)`
   - Compute initial vertical velocity: `vv = vz_final + g * t`
   - Compute predicted vertical displacement: `predicted = vv * t - 0.5 * g * t²`
   - Compute error: `error = predicted - actual_vertical_displacement`
   - If `|error| < EPSILON` → **SOLVED!**
   - Otherwise update: `t = t - error / (g * t)` ← this is Newton-Raphson!
4. If you exit the loop without converging, return a failure

### 3.3 Trajectory Simulation Method
Add a method that generates an array of positions along the predicted flight path. This is invaluable for debugging and visualization. The concept:
- Take a time step (e.g., 0.1 seconds)
- For each time step, compute position using: `x = x0 + vx*t`, `z = z0 + vz*t - 0.5*g*t²`
- Return the array of `Translation3d` positions

### 3.4 Multiple Solution Finder
Add a method that tries many different landing angles and finds the one requiring the least launch velocity. This is useful because there are many valid trajectories to hit the same target.

### 3.5 Yaw Normalization
When computing yaw angles, they can go outside the range [-180°, 180°]. You need normalization:
```java
while (yaw > Math.PI) yaw -= 2 * Math.PI;
while (yaw < -Math.PI) yaw += 2 * Math.PI;
```

### 3.6 Input Validation
The calculator should handle edge cases gracefully:
- Target directly above/below (horizontal distance ≈ 0)
- Negative flight time (impossible scenario)
- Numerical instability (derivative ≈ 0 in Newton-Raphson)

---

<a name="4-task-checklist"></a>
## 4. Task Checklist

### Phase 1: Fix the Foundation
- [ ] **Rename** `ballisticConstants` → use proper Java naming, restructure as an immutable ball properties class
- [ ] **Fix all getter methods** — remove parameters, make fields `final`, set values in constructors only
- [ ] **Fix** `BallisticConstraints` — add all six constraint boundaries, make immutable, add validation methods, add `defaults()` factory method
- [ ] **Expand** `BallisticSolution` — add all missing fields, add success/failure factory methods, add `toString()`
- [ ] **Create** `BallisticException` class
- [ ] **Clean imports** — remove `RobotContainer`, `RobotState`, unused `Pose2d`

### Phase 2: Fix the Math
- [ ] **Fix distance calculation** — correct the parentheses, extract into a helper method
- [ ] **Fix height calculation** — remove `Math.abs()`, use signed difference
- [ ] **Fix degrees/radians confusion** — decide on a convention, be consistent
- [ ] **Replace `Math.acos`** with `Math.atan2` for yaw calculation
- [ ] **Move gravity** to a `static final` constant in the calculator (9.81)

### Phase 3: Implement the Real Solver
- [ ] **Implement Newton-Raphson iteration** — this is the core algorithm (see Section 3.2)
- [ ] **Implement 3D velocity decomposition** — project vehicle velocity toward and perpendicular to target
- [ ] **Implement launch parameter extraction** — from the solved velocities, compute speed, pitch, yaw
- [ ] **Implement constraint validation** — check solution against launcher limits before returning
- [ ] **Write the main `calculate()` method** — tie everything together with a `Pose3d` input signature

### Phase 4: Add Features
- [ ] **Implement trajectory simulation** method
- [ ] **Implement multiple solution finder** (try many landing angles)
- [ ] **Implement minimum velocity finder**
- [ ] **Add yaw normalization**
- [ ] **Add edge case handling** (target too close, too far, directly above, etc.)

### Phase 5: Test
- [ ] **Run the existing test file** (`BallisticCalculatorTest.java`) — it currently tests the answer key, but once your code matches the interface, you can adapt it to test yours
- [ ] **Create your own test cases** — stationary launch, moving launch, lateral compensation
- [ ] **Verify** that a stationary robot aiming at a 5m target gives reasonable angles and velocities
- [ ] **Verify** that a moving robot's solution yaw compensates correctly

---

## Quick Reference: Key Formulas You'll Need

| What | Formula |
|------|---------|
| Horizontal distance | `sqrt(dx² + dy²)` where dx and dy are differences in X and Y |
| Yaw to target | `atan2(dy, dx)` |
| Velocity toward target | `vx·cos(yaw) + vy·sin(yaw)` |
| Lateral velocity | `-vx·sin(yaw) + vy·cos(yaw)` |
| Newton-Raphson update | `t_new = t - f(t) / f'(t)` |
| For this problem | `f(t) = -D·tan(θ) + 0.5·g·t² - Δh` and `f'(t) = g·t` |
| Launch speed | `sqrt(vh² + vv² + vlat²)` |
| Pitch angle | `atan2(vv, sqrt(vh² + vlat²))` |
| Yaw adjustment | `yaw_to_target + atan2(v_lateral, v_horizontal)` |

Where:
- `D` = horizontal distance to target
- `θ` = desired landing angle
- `Δh` = vertical displacement (target height minus launcher height)
- `vh, vv, vlat` = horizontal, vertical, and lateral launch velocity components (relative to vehicle)

---

**Remember:** The goal is understanding. Work through each step, draw diagrams, and test incrementally. Good luck! 🚀
