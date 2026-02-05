# **Building a Ballistic Trajectory Calculator with WPILib: A Learning Journey**

## **Project Overview**

In this project, you'll build a Java library that calculates how to shoot a ball from a moving robot into a stationary net. This is real-world physics applied to FRC robotics! You'll learn about projectile motion, coordinate systems, and how to compensate for movement **while using WPILib's geometry classes**.

**Your Mission**: Create a library that tells you:
- How fast to launch the ball
- What angle to aim (pitch and yaw)
- Where the ball will land

---

## **Part 1: Understanding WPILib's Geometry Classes**

### **What WPILib Provides**

WPILib includes powerful geometry classes you should use instead of creating your own [1][2]:

**`Pose3d`** - Represents position and orientation in 3D space [1][6]:
- Combines a `Translation3d` (x, y, z position) with a `Rotation3d` (orientation)
- Constructor: `new Pose3d(Translation3d translation, Rotation3d rotation)` [7]
- Alternative: `new Pose3d(x, y, z, rotation)` where x, y, z are in meters [6]

**`Translation3d`** - Represents a 3D position vector:
- Stores x, y, z coordinates
- Provides methods like `.getDistance()`, `.plus()`, `.minus()`

**`Rotation3d`** - Represents orientation in 3D [5]:
- Uses quaternions internally for robust rotation representation
- Note: Unlike 2D rotations, 3D rotations are not always commutative [5]
- Can be constructed from roll, pitch, yaw angles

**`Transform3d`** - Represents a transformation between poses [10]

### **WPILib Coordinate System**

WPILib uses the standard FRC field coordinate system:
- **X**: Forward (robot's perspective when looking down field)
- **Y**: Left (perpendicular to X)
- **Z**: Up

**Important**: This differs from some robotics conventions, so pay attention to the coordinate frame!

---

## **Part 2: Understanding Projectile Motion**

### **The Basic Physics**

Projectile motion follows a parabolic path - the object moves in a curve under gravity's influence. When you shoot a ball, it travels in an arc because gravity pulls it down while it moves forward.

**The fundamental equations of motion are**:

For any projectile at time `t`:
- **Horizontal position**: `x(t) = x₀ + v₀ₓ × t`
- **Vertical position**: `z(t) = z₀ + v₀ᵤ × t - ½g × t²`
- **Lateral position**: `y(t) = y₀ + v₀ᵧ × t`

Where:
- `x₀, y₀, z₀` = starting position (use `Translation3d.getX()`, etc.)
- `v₀ₓ, v₀ᵧ, v₀ᵤ` = initial velocity components
- `g` = gravity (9.81 m/s²)
- `t` = time in seconds

**Key Insight**: The horizontal motion is constant (no forces), but the vertical motion accelerates downward due to gravity.

---

## **Part 3: Setting Up Your Project**

### **Challenge 1: Create Your Class Structure**

Since WPILib provides `Pose3d`, you don't need to create basic geometry classes! Instead, focus on:

**Classes You WILL Create**:
- `BallProperties` - Stores diameter and mass of your game piece
- `LauncherConstraints` - Min/max velocity and angle limits for your mechanism
- `LaunchSolution` - Results containing launch parameters
- `BallisticCalculator` - The main computation engine

**Classes You WON'T Create** (use WPILib instead):
- ~~`Vector3d`~~ → Use `Translation3d`
- ~~`Pose3d`~~ → Use WPILib's `Pose3d` [1][6]
- ~~`Rotation3d`~~ → Use WPILib's `Rotation3d` [5]

### **Your First Task**: 

Create a `BallProperties` class that stores:
- Diameter in meters (use `double`)
- Mass in kilograms (use `double`)
- Include a static factory method `fromInches(double inches, double kg)` for convenience

---

## **Part 4: Working with WPILib Geometry**

### **Challenge 2: Extract Information from Pose3d**

Given a `Pose3d launcherPose` and a `Pose3d targetPose`, calculate:

1. **Relative position vector**: 
   ```java
   Translation3d relativePosition = targetPose.getTranslation()
       .minus(launcherPose.getTranslation());
   ```

2. **Horizontal distance** (distance in XY plane, ignoring Z):
   ```java
   double horizontalDist = Math.sqrt(
       relativePosition.getX() * relativePosition.getX() +
       relativePosition.getY() * relativePosition.getY()
   );
   ```

3. **Vertical displacement**:
   ```java
   double verticalDiff = relativePosition.getZ();
   ```

4. **Direction to target** (yaw angle):
   ```java
   double yawToTarget = Math.atan2(relativePosition.getY(), 
                                    relativePosition.getX());
   ```

**Practice**: If your robot is at (0, 0, 0.5) and the target is at (5, 2, 1.5), what are these values?

---

## **Part 5: Understanding the Moving Platform Challenge**

### **Velocity Composition**

Here's what makes this tricky: **You're launching from a moving robot!**

In FRC, you'll likely get your robot's velocity from odometry or a velocity estimator. This velocity needs to be combined with your launch velocity.

When your robot moves at velocity `V_robot` and you launch at velocity `V_launch` (relative to your robot), the ball's actual velocity relative to the field is:

**V_total = V_robot + V_launch**

### **Challenge 3: Velocity Decomposition**

You need to decompose your robot's velocity into components relative to the target direction:

**Given**:
- Robot velocity: `vx, vy, vz` (from odometry)
- Target direction: `θ_target` (calculated above)

**Calculate**:
```java
// Component toward target
double vToward = vx * Math.cos(θ_target) + vy * Math.sin(θ_target);

// Component perpendicular (lateral drift)
double vLateral = -vx * Math.sin(θ_target) + vy * Math.cos(θ_target);

// Vertical component
double vVertical = vz;
```

**Think About This**: If your robot is strafing right at 2 m/s while the target is straight ahead, what must you do to hit it?

---

## **Part 6: The Mathematics of Trajectory**

### **Solving for Launch Parameters**

You need to find the launch velocity and angles that satisfy TWO constraints:

1. **The ball must reach the target position**
2. **The ball must arrive at the desired landing angle**

### **The Core Equations**

Starting with the basic projectile equations, at landing time `t`:

**Horizontal constraint**:
```
horizontal_distance = v_horizontal × t
```

**Vertical constraint**:
```
vertical_displacement = v_vertical × t - ½ × g × t²
```

**Landing angle constraint**:

At landing, the ball's velocity has components:
- Horizontal: `v_h` (constant, no air resistance)
- Vertical: `v_z = v_vertical - g × t` (gravity has slowed/reversed it)

The landing angle θ relates these:
```
tan(θ) = |v_z| / v_h
```

### **Challenge 4: Derive the Solution**

Given these three equations and three unknowns (`v_horizontal`, `v_vertical`, `t`), work through solving for them.

**Hint**: 
1. From horizontal equation: `v_h = distance / t`
2. From landing angle: `v_z = -v_h × tan(θ)` (negative = descending)
3. From velocity equation: `v_vertical = v_z + g × t`
4. Substitute into vertical displacement equation and solve for `t`

This requires **iteration** because the equations are coupled!

---

## **Part 7: The Iterative Solution Method**

### **Newton-Raphson Iteration**

Since we can't solve this algebraically in one step, use an iterative numerical method:

1. **Make an initial guess** for flight time `t`
2. **Calculate** what the vertical displacement would be with that `t`
3. **Compare** to the actual target height
4. **Adjust** `t` based on the error
5. **Repeat** until the error is tiny (< 0.000001 meters)

**The Update Formula**:

```java
double t = initialGuess;
for (int i = 0; i < MAX_ITERATIONS; i++) {
    double vh = horizontalDistance / t;
    double vz_final = -vh * Math.tan(landingAngle);
    double vv = vz_final + GRAVITY * t;
    
    double calculatedZ = vv * t - 0.5 * GRAVITY * t * t;
    double error = calculatedZ - actualVerticalDisplacement;
    
    if (Math.abs(error) < EPSILON) {
        // Found solution!
        break;
    }
    
    // Newton-Raphson update
    double derivative = GRAVITY * t;
    t = t - error / derivative;
}
```

### **Challenge 5: Implement the Solver**

Create a method `calculateTrajectory()` that:
- Takes: `Pose3d launcher`, `Translation3d robotVelocity`, `Translation3d target`, `double landingAngleDeg`
- Returns: A `LaunchSolution` object with velocity, pitch, yaw, and flight time
- Uses the iteration loop above
- Handles edge cases (target too close, too far, directly above, etc.)

---

## **Part 8: Converting Back to Launch Parameters**

### **Challenge 6: Extract Pitch and Yaw**

Once you've solved for the velocity components, you need to convert them to angles your launcher mechanism can use:

**Given velocity components** (relative to launcher, after subtracting robot velocity):
```java
double vLaunchForward = /* calculated */;
double vLaunchLateral = /* calculated */;
double vLaunchVertical = /* calculated */;
```

**Calculate launch speed**:
```java
double launchSpeed = Math.sqrt(
    vLaunchForward * vLaunchForward +
    vLaunchLateral * vLaunchLateral +
    vLaunchVertical * vLaunchVertical
);
```

**Calculate pitch** (elevation angle):
```java
double horizontalComponent = Math.sqrt(
    vLaunchForward * vLaunchForward +
    vLaunchLateral * vLaunchLateral
);
double pitchAngle = Math.atan2(vLaunchVertical, horizontalComponent);
```

**Calculate yaw** (horizontal direction):
```java
double yawAngle = targetYaw + Math.atan2(vLaunchLateral, vLaunchForward);
```

You can then create a `Rotation3d` from these angles [5]:
```java
Rotation3d launchRotation = new Rotation3d(0, pitchAngle, yawAngle);
```

---

## **Part 9: Creating the LaunchSolution Class**

### **Challenge 7: Design Your Result Object**

Create a `LaunchSolution` class that stores:

**If solution found**:
- `boolean solutionFound` = true
- `double launchVelocity` (m/s)
- `double pitchAngleDegrees`
- `double yawAngleDegrees`
- `double flightTime` (seconds)
- `Rotation3d launchRotation` (using WPILib's class) [5]
- `Pose3d predictedLandingPose` (using WPILib's class) [1]

**If solution NOT found**:
- `boolean solutionFound` = false
- `String failureReason` (e.g., "Target too far", "Impossible angle")

Include static factory methods:
```java
public static LaunchSolution success(/* parameters */) { ... }
public static LaunchSolution failure(String reason) { ... }
```

---

## **Part 10: Validation and Constraints**

### **Challenge 8: Implement Constraint Checking**

Create a `LauncherConstraints` class with:
- `minVelocity` and `maxVelocity` (your launcher's speed limits)
- `minPitch` and `maxPitch` (mechanism angle limits)
- `minYaw` and `maxYaw` (turret rotation limits)

After calculating a solution, validate it:
```java
if (launchSpeed < constraints.minVelocity || 
    launchSpeed > constraints.maxVelocity) {
    return LaunchSolution.failure(
        "Required velocity outside launcher limits");
}
```

**Real-World Consideration**: An FRC shooter might only be able to launch between 5-25 m/s, and your turret might only rotate ±90 degrees!

---

## **Part 11: Testing with WPILib**

### **Challenge 9: Create Test Scenarios**

Write test methods using WPILib classes [1][6]:

**Test 1: Stationary Robot**
```java
Pose3d robotPose = new Pose3d(0, 0, 0.5, new Rotation3d());
Translation3d robotVel = new Translation3d();  // Zero velocity
Pose3d targetPose = new Pose3d(5, 0, 1.0, new Rotation3d());
```

**Test 2: Moving Forward**
```java
Pose3d robotPose = new Pose3d(0, 0, 0.5, new Rotation3d());
Translation3d robotVel = new Translation3d(3, 0, 0);  // 3 m/s forward
Pose3d targetPose = new Pose3d(8, 0, 1.0, new Rotation3d());
```

**Test 3: Strafing Robot**
```java
Pose3d robotPose = new Pose3d(0, 0, 0.5, new Rotation3d());
Translation3d robotVel = new Translation3d(2, 1.5, 0);  // Moving diagonally
Pose3d targetPose = new Pose3d(6, 0, 1.0, new Rotation3d());
```

For each test, verify:
- Does the solution exist?
- Is the pitch angle reasonable?
- Does the yaw compensate for robot motion correctly?

---

## **Part 12: Trajectory Simulation**

### **Challenge 10: Visualize the Path**

Create a method that simulates the trajectory step-by-step:

```java
public Pose3d[] simulateTrajectory(
    Pose3d startPose,
    Translation3d initialVelocity,
    LaunchSolution solution,
    double timeStep
) {
    int numSteps = (int)(solution.getFlightTime() / timeStep) + 1;
    Pose3d[] trajectory = new Pose3d[numSteps];
    
    for (int i = 0; i < numSteps; i++) {
        double t = i * timeStep;
        double x = startPose.getX() + initialVelocity.getX() * t;
        double y = startPose.getY() + initialVelocity.getY() * t;
        double z = startPose.getZ() + initialVelocity.getZ() * t 
                   - 0.5 * GRAVITY * t * t;
        
        trajectory[i] = new Pose3d(x, y, z, new Rotation3d());
    }
    
    return trajectory;
}
```

You can use this array of `Pose3d` objects [1][7] to:
- Display the trajectory in Shuffleboard or Glass
- Verify your calculations
- Debug why shots are missing

---

## **Part 13: Integration with Robot Code**

### **How This Fits Into FRC**

In your robot code, you'll likely use this with:

**Odometry** to get robot pose [8]:
```java
Pose3d currentPose = poseEstimator.getEstimatedPosition3d();
```

**Velocity estimation**:
```java
ChassisSpeeds robotSpeeds = /* from your drive subsystem */;
Translation3d robotVelocity = new Translation3d(
    robotSpeeds.vxMetersPerSecond,
    robotSpeeds.vyMetersPerSecond,
    0  // Usually no vertical velocity
);
```

**Vision-detected target**:
```java
Pose3d targetPose = visionSubsystem.getTargetPose();
```

**Then calculate**:
```java
LaunchSolution shot = calculator.calculate(
    currentPose,
    robotVelocity,
    targetPose.getTranslation(),
    45.0  // Landing angle in degrees
);

if (shot.isSolutionFound()) {
    shooterSubsystem.setVelocity(shot.getLaunchVelocity());
    turretSubsystem.setAngle(shot.getYawAngleDegrees());
    hoodSubsystem.setAngle(shot.getPitchAngleDegrees());
}
```

---

## **Part 14: Advanced Challenges**

Once you have the basic calculator working:

### **Extension 1: Minimum Velocity Finder**
Calculate solutions for multiple landing angles (30° to 70°) and select the one requiring minimum launch velocity - saves energy and reduces variability!

### **Extension 2: Lead Time Calculation**
If your vision system detects the target is moving (like in 2020 Power Port oscillation), extend the math to predict where it will be when the ball arrives.

### **Extension 3: Field-Relative Calculations**
Use `Transform3d` [10] to handle when your robot is rotated relative to the field and needs to shoot at an angle.

### **Extension 4: SmartDashboard Integration**
Send trajectory points to NetworkTables for real-time visualization of predicted shot paths.

---

## **Learning Checkpoints**

By completing this project, you will have learned:

✓ **WPILib Geometry**: `Pose3d`, `Translation3d`, `Rotation3d` classes [1][5][6][7]  
✓ **Physics**: Projectile motion and parabolic trajectories  
✓ **Mathematics**: Trigonometry, vectors, numerical iteration  
✓ **FRC Integration**: Odometry, pose estimation, command-based programming [8]  
✓ **Engineering**: Constraint handling, edge cases, testing  
✓ **Problem Solving**: Breaking complex problems into solvable pieces

---

## **Getting Help**

When you get stuck:

- **Check WPILib docs**: The official documentation has excellent examples
- **Draw diagrams**: Sketch the coordinate system and vectors
- **Test incrementally**: Build one piece at a time
- **Use SmartDashboard**: Print intermediate values to debug
- **Ask mentors**: The physics and math are challenging - that's the point!

Remember: The goal is to **understand** the mathematics and physics, not just get working code. Take time to work through the derivations and convince yourself the equations make sense!

**Good luck, and may your shots be accurate!** 🎯
