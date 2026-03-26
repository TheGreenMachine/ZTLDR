package com.team1816.season.subsystems;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BrentSolver;

/**
 * BallisticTester provides utilities to calculate the required launch velocity, pitch,
 * and yaw adjustment to hit a moving target at a specific impact angle.
 *
 * It solves the ballistic trajectory equations using a quartic root-finding approach
 * (Brent's Method) to account for target velocity (Vtx, Vty) and gravity.
 */
public class BallisticTester {

    /** Gravitational acceleration in m/s^2. */
    private static final double G = 9.81;

    /**
     * Reusable BrentSolver instance to minimize object allocation during
     * high-frequency control loops.
     */
    private static final BrentSolver SOLVER = new BrentSolver();

    /**
     * Entry point for running unit tests to validate the solver across
     * various ranges and target velocities.
     */
    public static void main(String[] args) {
        System.out.println("=== Ballistic Equation Unit Tests ===\n");

        // Format: Name, Target X, Target Y, Target Z, Impact Angle, Target Vx, Target Vy
        runTest("Short Range Low", 2.15, 0.0, 0.45, -35.2, 0.5, -0.2);
        runTest("Mid Range High", 5.60, 0.0, 1.85, -55.0, -1.2, 0.8);
        runTest("Long Range Flat", 9.20, 0.0, 0.10, -65.4, 1.8, 1.5);
        runTest("Fast Moving Target 1", 4.30, 0.0, 1.20, -42.8, -1.9, -1.9);
        runTest("Fast Moving Target 2", 7.80, 0.0, 0.95, -58.2, 2.0, -0.5);
        runTest("Steep Angle Near", 1.50, 0.0, 1.90, -68.5, 0.1, 0.1);
        runTest("Shallow Angle Far", 8.45, 0.0, 0.60, -31.5, -0.7, 1.2);
        // ... (Additional test cases omitted for brevity)
        runTest("Test Case 30", 5.00, 0.0, 0.00, -45.0, 0.0, 0.0);
    }

    /**
     * Warmup method to be called during robotInit().
     * Forces the JIT (Just-In-Time) compiler to optimize the solver's machine code
     * before the match starts, preventing latency spikes during the first few shots.
     */
    public static void warmup() {
        for (int i = 0; i < 2000; i++) {
            solve(2.15, 0, 0.45, 0.5, -0.2, -35.2);
        }
    }

    /**
     * Calculates the necessary launch parameters to hit a moving target.
     *
     * @param tx       Target X position (m) relative to shooter.
     * @param ty       Target Y position (m) relative to shooter.
     * @param tz       Target Z height (m) relative to shooter.
     * @param vtx      Target velocity in X (m/s).
     * @param vty      Target velocity in Y (m/s).
     * @param thetaDeg Desired impact angle at the target in degrees (usually negative).
     * @return double[] {Launch Velocity (m/s), Pitch Angle (deg), Yaw Offset (deg), Time of Flight (s)}
     */
    public static double[] solve(double tx, double ty, double tz, double vtx, double vty, double thetaDeg) {
        double rad = Math.toRadians(thetaDeg);
        double A = Math.tan(rad);

        // Coefficients for the quartic equation: f(T) = aT^4 + cT^2 + dT + e
        // Derived from substituting kinematic equations into the impact angle constraint.
        final double a = 0.25 * G * G;
        final double c = -(G * tz + A * A * (vtx * vtx + vty * vty));
        final double d = -2 * A * A * (tx * vtx + ty * vty);
        final double e = tz * tz - A * A * (tx * tx + ty * ty);

        // Functional interface for the quartic equation to find Time of Flight (T)
        UnivariateFunction quartic = t -> a * t * t * t * t + c * t * t + d * t + e;

        double T = -1;
        double step = 0.1;

        // Bracketing search: find a sign change in the function to locate a root
        for (double t = 0.05; t < 4.0; t += step) {
            if (quartic.value(t) * quartic.value(t + step) < 0) {
                // Use Brent's method for high-accuracy root finding within the bracket
                T = SOLVER.solve(50, quartic, t, t + step);
                break;
            }
        }

        // Return zeros if no valid time of flight is found (out of range)
        if (T < 0) return new double[]{0, 0, 0, 0};

        // Calculate intercept coordinates based on target motion during flight
        double ix = tx + vtx * T;
        double iy = ty + vty * T;
        double R = Math.sqrt(ix * ix + iy * iy); // Horizontal distance to intercept

        // Decompose velocities
        double vh = R / T;                          // Required horizontal velocity
        double vpz = (tz + 0.5 * G * T * T) / T;    // Required vertical velocity

        return new double[]{
            Math.sqrt(vh * vh + vpz * vpz),                           // Magnitude (Resultant Velocity)
            Math.toDegrees(Math.atan2(vpz, vh)),                     // Launch Pitch
            Math.toDegrees(Math.atan2(iy, ix) - Math.atan2(ty, tx)), // Yaw adjustment for lead
            T                                                        // Time of Flight
        };
    }

    /**
     * Executes a single test case and prints results to console.
     */
    private static void runTest(String name, double tx, double ty, double tz, double deg, double vx, double vy) {
        double start = System.nanoTime() / 1_000_000.0;
        double[] result = solve(tx, ty, tz, vx, vy, deg);
        double end = System.nanoTime() / 1_000_000.0;

        System.out.printf("[%s]\n", name);
        System.out.printf("  Inputs: targetX=%.2fm, targetY=0, targetZ=%.2fm, targetAngle=%.1fdeg, targetVx=%.1fm/s, targetVy=%.1fm/s\n", tx, tz, deg, vx, vy);
        System.out.printf("  OUTPUT -> Velocity: %.4f m/s\n", result[0]);
        System.out.printf("  OUTPUT -> Pitch Adj: %.4f deg\n", result[1]);
        System.out.printf("  OUTPUT -> Yaw Adj:   %.4f deg\n", result[2]);
        System.out.printf("  OUTPUT -> Time: %.4f sec\n", result[3]);
        System.out.printf("  OUTPUT -> Solve Time: %.4f ms\n\n", end - start);
    }
}
