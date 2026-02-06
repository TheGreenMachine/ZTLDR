package com.team1816.lib.util.ballisticCalc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
/**
 * Main ballistic trajectory calculator.
 *
 * Calculates launch parameters to hit a stationary target from a moving vehicle,
 * compensating for vehicle velocity and achieving a desired landing angle.
 *
 * Coordinate system:
 * - X: forward (positive) / backward (negative)
 * - Y: up (positive) / down (negative)
 * - Z: right (positive) / left (negative)
 *
 * Uses standard projectile motion equations:
 * x(t) = x0 + vx*t
 * y(t) = y0 + vy*t - 0.5*g*t²
 * z(t) = z0 + vz*t
 */
public class BallisticCalculator {

    private static final double GRAVITY = 9.81;  // m/s²
    private static final double EPSILON = 1e-6;
    private static final int MAX_ITERATIONS = 100;

    private final BallProperties ballProperties;
    private final LauncherConstraints constraints;

    /**
     * Creates a ballistic calculator with specified ball properties and launcher constraints.
     */
    public BallisticCalculator(BallProperties ballProperties, LauncherConstraints constraints) {
        this.ballProperties = ballProperties;
        this.constraints = constraints;
    }

    /**
     * Creates a ballistic calculator with default launcher constraints.
     */
    public BallisticCalculator(BallProperties ballProperties) {
        this(ballProperties, LauncherConstraints.defaults());
    }

    /**
     * Calculates the launch parameters to hit a target.
     *
     * @param launcherPose Current pose of the launcher (position and orientation)
     * @param vehicleVelocity Current velocity of the vehicle (vx, vy, vz) in m/s
     * @param targetPosition Position of the target in world coordinates
     * @param desiredLandingAngleDegrees Desired angle of descent at target (positive = steep)
     * @return LaunchSolution containing launch parameters or failure reason
     */
    public LaunchSolution calculate(
            Pose3d launcherPose,
            Translation3d vehicleVelocity,
            Translation3d targetPosition,
            double desiredLandingAngleDegrees) {

        double desiredLandingAngle = Math.toRadians(desiredLandingAngleDegrees);

        // Calculate relative position to target
        Translation3d launcherPos = launcherPose.getTranslation();
        Translation3d relativeTarget = targetPosition.minus(launcherPos);

        // Horizontal distance and direction to target
        double horizontalDistance = Math.sqrt(
            relativeTarget.getX() * relativeTarget.getX() +
            relativeTarget.getY() * relativeTarget.getY()
        );

        if (horizontalDistance < EPSILON) {
            return LaunchSolution.failure("Target is directly above/below launcher");
        }

        // Vertical displacement (positive = target is higher)
        double verticalDisplacement = relativeTarget.getZ();

        // Calculate yaw angle to target (world frame)
        double yawToTarget = Math.atan2(relativeTarget.getY(), relativeTarget.getX());

        // Project vehicle velocity onto the horizontal plane toward target
        // This is the component of vehicle velocity in the direction of the target
        double vehicleHorizontalSpeed = vehicleVelocity.getX() * Math.cos(yawToTarget) +
                                        vehicleVelocity.getY() * Math.sin(yawToTarget);
        double vehicleVerticalSpeed = vehicleVelocity.getZ();

        // Perpendicular component of vehicle velocity (for yaw compensation)
        double vehicleLateralSpeed = -vehicleVelocity.getX() * Math.sin(yawToTarget) +
                                      vehicleVelocity.getY() * Math.cos(yawToTarget);

        // Now solve for launch velocity that achieves desired landing angle
        // At landing: tan(landingAngle) = -vy_final / vh_final
        // where vy_final = vy_launch - g*t (negative at landing)
        // and vh_final = vh_launch (constant, no air resistance)

        LaunchSolution solution = solveForLandingAngle(
            horizontalDistance,
            verticalDisplacement,
            desiredLandingAngle,
            vehicleHorizontalSpeed,
            vehicleVerticalSpeed,
            vehicleLateralSpeed,
            yawToTarget,
            targetPosition
        );

        return solution;
    }

    /**
     * Solves for launch parameters that achieve the desired landing angle.
     */
    private LaunchSolution solveForLandingAngle(
            double horizontalDistance,
            double verticalDisplacement,
            double desiredLandingAngle,
            double vehicleHorizontalSpeed,
            double vehicleVerticalSpeed,
            double vehicleLateralSpeed,
            double yawToTarget,
            Translation3d targetPosition) {

        // We need to find launch velocity (v) and launch angle (theta) such that:
        // 1. The ball lands at the target position
        // 2. The ball arrives at the desired landing angle
        //
        // Total velocity components (world frame, in direction of target):
        // vh_total = v*cos(theta) + vehicleHorizontalSpeed  (horizontal toward target)
        // vv_total = v*sin(theta) + vehicleVerticalSpeed    (vertical)
        //
        // At time t, position:
        // horizontal: vh_total * t = horizontalDistance
        // vertical: vv_total * t - 0.5*g*t² = verticalDisplacement
        //
        // At landing, vertical velocity:
        // vy_final = vv_total - g*t
        //
        // Landing angle constraint:
        // tan(desiredLandingAngle) = |vy_final| / vh_total = (g*t - vv_total) / vh_total

        // Iterate to find a solution
        // Start with an initial guess for flight time based on a 45-degree launch
        double tGuess = Math.sqrt(2 * horizontalDistance / GRAVITY);

        for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
            double t = tGuess;

            if (t <= 0) {
                return LaunchSolution.failure("Invalid flight time calculated");
            }

            // From horizontal distance: vh_total = horizontalDistance / t
            double vh_total = horizontalDistance / t;

            // From landing angle: vy_final = -vh_total * tan(landingAngle)
            // (negative because ball is descending)
            double vy_final = -vh_total * Math.tan(desiredLandingAngle);

            // From vy_final = vv_total - g*t:
            double vv_total = vy_final + GRAVITY * t;

            // Check vertical displacement constraint:
            // verticalDisplacement = vv_total * t - 0.5 * g * t²
            double calculatedVertical = vv_total * t - 0.5 * GRAVITY * t * t;
            double verticalError = calculatedVertical - verticalDisplacement;

            if (Math.abs(verticalError) < EPSILON) {
                // Found a solution! Now extract launch parameters

                // Launch velocity components (relative to vehicle, in target direction frame)
                double vLaunchHorizontal = vh_total - vehicleHorizontalSpeed;
                double vLaunchVertical = vv_total - vehicleVerticalSpeed;
                double vLaunchLateral = -vehicleLateralSpeed; // Compensate for lateral drift

                // Calculate total launch speed and angles
                double launchSpeed = Math.sqrt(
                    vLaunchHorizontal * vLaunchHorizontal +
                    vLaunchVertical * vLaunchVertical +
                    vLaunchLateral * vLaunchLateral
                );

                double pitchAngle = Math.atan2(vLaunchVertical,
                    Math.sqrt(vLaunchHorizontal * vLaunchHorizontal + vLaunchLateral * vLaunchLateral));

                // Adjust yaw for lateral compensation
                double yawAngle = yawToTarget + Math.atan2(vLaunchLateral, vLaunchHorizontal);

                // Normalize yaw to [-π, π]
                while (yawAngle > Math.PI) yawAngle -= 2 * Math.PI;
                while (yawAngle < -Math.PI) yawAngle += 2 * Math.PI;

                // Validate against constraints
                if (!constraints.isVelocityValid(launchSpeed)) {
                    return LaunchSolution.failure(String.format(
                        "Required launch velocity %.2f m/s is outside constraints [%.2f, %.2f]",
                        launchSpeed, constraints.getMinLaunchVelocity(), constraints.getMaxLaunchVelocity()));
                }

                if (!constraints.isPitchValid(pitchAngle)) {
                    return LaunchSolution.failure(String.format(
                        "Required pitch angle %.2f° is outside constraints [%.2f°, %.2f°]",
                        Math.toDegrees(pitchAngle),
                        Math.toDegrees(constraints.getMinPitchAngle()),
                        Math.toDegrees(constraints.getMaxPitchAngle())));
                }

                // Calculate world-frame launch velocity vector
                Translation3d launchVelocityVector = new Translation3d(
                    vLaunchHorizontal * Math.cos(yawToTarget) - vLaunchLateral * Math.sin(yawToTarget),
                    vLaunchVertical,
                    vLaunchHorizontal * Math.sin(yawToTarget) + vLaunchLateral * Math.cos(yawToTarget)
                );

                return LaunchSolution.success(
                    launchSpeed,
                    pitchAngle,
                    yawAngle,
                    t,
                    launchVelocityVector,
                    targetPosition,
                    desiredLandingAngle
                );
            }

            // Newton-Raphson iteration to improve t
            // f(t) = vv_total * t - 0.5 * g * t² - verticalDisplacement
            // where vv_total = -vh_total * tan(angle) + g * t = -(D/t) * tan(angle) + g * t
            // f(t) = (-(D/t) * tan(angle) + g * t) * t - 0.5 * g * t² - verticalDisplacement
            // f(t) = -D * tan(angle) + g * t² - 0.5 * g * t² - verticalDisplacement
            // f(t) = -D * tan(angle) + 0.5 * g * t² - verticalDisplacement
            // f'(t) = g * t

            double tanAngle = Math.tan(desiredLandingAngle);
            double f = -horizontalDistance * tanAngle + 0.5 * GRAVITY * t * t - verticalDisplacement;
            double fPrime = GRAVITY * t;

            if (Math.abs(fPrime) < EPSILON) {
                return LaunchSolution.failure("Numerical instability in solution");
            }

            tGuess = t - f / fPrime;

            // Ensure t stays positive
            if (tGuess <= 0) {
                tGuess = t / 2;
            }
        }

        return LaunchSolution.failure("Failed to converge on a solution after " + MAX_ITERATIONS + " iterations");
    }

    /**
     * Simulates the trajectory and returns positions at regular time intervals.
     * Useful for visualization and debugging.
     *
     * @param launcherPosition Starting position
     * @param vehicleVelocity Vehicle velocity at launch
     * @param solution The launch solution to simulate
     * @param timeStep Time between samples in seconds
     * @return Array of positions along the trajectory
     */
    public Translation3d[] simulateTrajectory(
            Translation3d launcherPosition,
            Translation3d vehicleVelocity,
            LaunchSolution solution,
            double timeStep) {

        if (!solution.isSolutionFound()) {
            return new Translation3d[0];
        }

        int numSteps = (int) Math.ceil(solution.getFlightTime() / timeStep) + 1;
        Translation3d[] trajectory = new Translation3d[numSteps];

        // Total initial velocity = launch velocity (relative) + vehicle velocity
        Translation3d totalVelocity = solution.getLaunchVelocityVector().plus(vehicleVelocity);

        for (int i = 0; i < numSteps; i++) {
            double t = Math.min(i * timeStep, solution.getFlightTime());

            double x = launcherPosition.getX() + totalVelocity.getX() * t;
            double y = launcherPosition.getX() + totalVelocity.getX() * t - 0.5 * GRAVITY * t * t;
            double z = launcherPosition.getY() + totalVelocity.getY() * t;

            trajectory[i] = new Translation3d(x, y, z);
        }

        return trajectory;
    }

    /**
     * Calculates multiple solutions with different landing angles.
     * Useful for finding the best trajectory among multiple options.
     *
     * @param launcherPose Current pose of the launcher
     * @param vehicleVelocity Current velocity of the vehicle
     * @param targetPosition Position of the target
     * @param minLandingAngleDegrees Minimum landing angle to try
     * @param maxLandingAngleDegrees Maximum landing angle to try
     * @param steps Number of angles to try
     * @return Array of solutions (some may have solutionFound = false)
     */
    public LaunchSolution[] calculateMultipleSolutions(
            Pose3d launcherPose,
            Translation3d vehicleVelocity,
            Translation3d targetPosition,
            double minLandingAngleDegrees,
            double maxLandingAngleDegrees,
            int steps) {

        LaunchSolution[] solutions = new LaunchSolution[steps];
        double angleStep = (maxLandingAngleDegrees - minLandingAngleDegrees) / (steps - 1);

        for (int i = 0; i < steps; i++) {
            double angle = minLandingAngleDegrees + i * angleStep;
            solutions[i] = calculate(launcherPose, vehicleVelocity, targetPosition, angle);
        }

        return solutions;
    }

    /**
     * Finds the solution that requires the minimum launch velocity.
     */
    public LaunchSolution findMinimumVelocitySolution(
            Pose3d launcherPose,
            Translation3d vehicleVelocity,
            Translation3d targetPosition,
            double minLandingAngleDegrees,
            double maxLandingAngleDegrees) {

        LaunchSolution[] solutions = calculateMultipleSolutions(
            launcherPose, vehicleVelocity, targetPosition,
            minLandingAngleDegrees, maxLandingAngleDegrees, 50
        );

        LaunchSolution best = null;
        for (LaunchSolution sol : solutions) {
            if (sol.isSolutionFound()) {
                if (best == null || sol.getLaunchVelocity() < best.getLaunchVelocity()) {
                    best = sol;
                }
            }
        }

        return best != null ? best : LaunchSolution.failure("No valid solution found in angle range");
    }

    public BallProperties getBallProperties() {
        return ballProperties;
    }

    public LauncherConstraints getConstraints() {
        return constraints;
    }
}
