package com.team1816.lib.ballisticCalc;

import com.team1816.lib.util.ballisticCalc.BallProperties;
import com.team1816.lib.util.ballisticCalc.BallisticCalculator;
import com.team1816.lib.util.ballisticCalc.LaunchSolution;
import com.team1816.lib.util.ballisticCalc.LauncherConstraints;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Test cases for BallisticCalculator.
 */
public class BallisticCalculatorTest {

    public static void main(String[] args) {
        System.out.println("=== Ballistic Calculator Tests ===\n");

        testStationaryLaunch();
        testMovingVehicleLaunch();
        testLateralMovement();
        testTrajectorySimulation();
        testMinimumVelocitySolution();
        testEdgeCases();

        System.out.println("\n=== All Tests Complete ===");
    }

    static void testStationaryLaunch() {
        System.out.println("Test 1: Stationary launch to target 5m away");

        BallProperties ball = BallProperties.fromInches(6, 0.5);
        BallisticCalculator calc = new BallisticCalculator(ball);

        Pose3d launcherPose = new Pose3d(0, 0, 1, new Rotation3d());  // 1m off ground
        Translation3d vehicleVelocity = new Translation3d();  // Stationary
        Translation3d target = new Translation3d(5, 1, 0);   // 5m forward, same height

        LaunchSolution solution = calc.calculate(launcherPose, vehicleVelocity, target, 45);

        System.out.println("  Ball: " + ball);
        System.out.println("  Target: " + target);
        System.out.println("  Solution: " + solution);
        System.out.println();
    }

    static void testMovingVehicleLaunch() {
        System.out.println("Test 2: Moving vehicle (5 m/s forward) to target 8m away");

        BallProperties ball = BallProperties.fromInches(6, 0.5);
        BallisticCalculator calc = new BallisticCalculator(ball);

        Pose3d launcherPose = new Pose3d(0, 0, 1, new Rotation3d());
        Translation3d vehicleVelocity = new Translation3d(5, 0, 0);  // 5 m/s forward
        Translation3d target = new Translation3d(8, 0.5, 0);         // 8m forward, slightly lower

        LaunchSolution solution = calc.calculate(launcherPose, vehicleVelocity, target, 60);

        System.out.println("  Vehicle velocity: " + vehicleVelocity);
        System.out.println("  Target: " + target);
        System.out.println("  Solution: " + solution);
        System.out.println();
    }

    static void testLateralMovement() {
        System.out.println("Test 3: Vehicle moving laterally while shooting forward");

        BallProperties ball = BallProperties.fromInches(6, 0.5);
        BallisticCalculator calc = new BallisticCalculator(ball);

        Pose3d launcherPose = new Pose3d(0, 0, 1, new Rotation3d());
        Translation3d vehicleVelocity = new Translation3d(2, 0, 3);  // Moving forward and right
        Translation3d target = new Translation3d(6, 0.5, 0);         // Target is straight ahead

        LaunchSolution solution = calc.calculate(launcherPose, vehicleVelocity, target, 50);

        System.out.println("  Vehicle velocity: " + vehicleVelocity + " (moving right)");
        System.out.println("  Target: " + target);
        System.out.println("  Solution: " + solution);
        System.out.println("  Note: Yaw should compensate by aiming left of target");
        System.out.println();
    }

    static void testTrajectorySimulation() {
        System.out.println("Test 4: Trajectory simulation");

        BallProperties ball = BallProperties.fromInches(6, 0.5);
        BallisticCalculator calc = new BallisticCalculator(ball);

        Pose3d launcherPose = new Pose3d(0, 1, 0, new Rotation3d());
        Translation3d vehicleVelocity = new Translation3d(3, 0, 0);
        Translation3d target = new Translation3d(10, 0.5, 0);

        LaunchSolution solution = calc.calculate(launcherPose, vehicleVelocity, target, 55);

        if (solution.isSolutionFound()) {
            Translation3d[] trajectory = calc.simulateTrajectory(
                launcherPose.getTranslation(), vehicleVelocity, solution, 0.1);

            System.out.println("  Trajectory points (every 0.1s):");
            for (int i = 0; i < trajectory.length; i++) {
                if (i % 3 == 0 || i == trajectory.length - 1) {  // Print every 3rd point
                    System.out.printf("    t=%.1fs: %s%n", i * 0.1, trajectory[i]);
                }
            }
        }
        System.out.println();
    }

    static void testMinimumVelocitySolution() {
        System.out.println("Test 5: Find minimum velocity solution");

        BallProperties ball = BallProperties.fromInches(6, 0.5);
        BallisticCalculator calc = new BallisticCalculator(ball);

        Pose3d launcherPose = new Pose3d(0, 0, 1, new Rotation3d());
        Translation3d vehicleVelocity = new Translation3d();
        Translation3d target = new Translation3d(7, 0.5, 2);  // 7m forward, 2m right

        LaunchSolution solution = calc.findMinimumVelocitySolution(
            launcherPose, vehicleVelocity, target, 30, 70);

        System.out.println("  Target: " + target);
        System.out.println("  Minimum velocity solution: " + solution);
        System.out.println();
    }

    static void testEdgeCases() {
        System.out.println("Test 6: Edge cases");

        BallProperties ball = BallProperties.fromInches(6, 0.5);
        LauncherConstraints constraints = new LauncherConstraints(1, 15, -10, 80, -180, 180);
        BallisticCalculator calc = new BallisticCalculator(ball, constraints);

        Pose3d launcherPose = new Pose3d(0, 1, 0, new Rotation3d());
        Translation3d vehicleVelocity = new Translation3d();

        // Target too far for max velocity
        Translation3d farTarget = new Translation3d(50, 0, 0);
        LaunchSolution farSolution = calc.calculate(launcherPose, vehicleVelocity, farTarget, 45);
        System.out.println("  Far target (50m): " + farSolution);

        // Target directly above
        Translation3d aboveTarget = new Translation3d(0.001, 5, 0);
        LaunchSolution aboveSolution = calc.calculate(launcherPose, vehicleVelocity, aboveTarget, 85);
        System.out.println("  Target above: " + aboveSolution);

        System.out.println();
    }
}
