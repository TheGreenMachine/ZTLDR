package com.team1816.lib.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1816.lib.Singleton;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static com.team1816.lib.Singleton.factory;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
public class Swerve extends SubsystemBase implements ITestableSubsystem {

    private final String NAME = "drivetrain";

    private final IDrivetrain drivetrain;

    private final CommandXboxController controller;
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);    // forward/back
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);    // strafe
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);  // rotation
    private static double maxAngularRate = 0;

    private ActualState wantedState = ActualState.IDLING;
    private ActualState previousWantedState = ActualState.IDLING;

    public Swerve(CommandXboxController controller) {
        // TODO: This Singleton.get stuff is a temporary workaround until I fix a bug with the
        //  static factory member from the Singleton not initializing properly for the first
        //  subsystem that is created.
        drivetrain = Singleton.get(RobotFactory.class).getSwerveDrivetrain(NAME);

        this.controller = controller;
        var kinematics = drivetrain.getKinematicsConfig();
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // setup teleop drivetrain command
        maxAngularRate = RotationsPerSecond.of(kinematics.maxAngularRate).in(RadiansPerSecond);
    }

    public enum ActualState {
        MANUAL_DRIVING,
        AUTOMATIC_DRIVING,
        IDLING
    }

    @Override
    public void periodic() {
        readFromHardware();
        applyStates();
    }

    private SwerveRequest GetSwerverCommand(SwerveRequest.FieldCentric drive) {

        // 1. Get raw joystick values (-1.0 to +1.0)
        double rawX    = -controller.getLeftY();    // forward/back  (negative because forward is usually negative Y)
        double rawY    = -controller.getLeftX();    // strafe left/right
        double rawRot  = -controller.getRightX();   // rotation

        // 2. Deadband (remove drift)
        rawX   = Math.abs(rawX)   < 0.08 ? 0 : rawX;
        rawY   = Math.abs(rawY)   < 0.08 ? 0 : rawY;
        rawRot = Math.abs(rawRot) < 0.08 ? 0 : rawRot;

        // 3. Cube the inputs → insane precision at low speed, full power at full stick
        double x    = rawX   * rawX   * rawX;
        double y    = rawY   * rawY   * rawY;
        double rot  = rawRot * rawRot * rawRot;

        // 4. Slew rate limit → buttery smooth acceleration
        x   = xLimiter.calculate(x);
        y   = yLimiter.calculate(y);
        rot = rotLimiter.calculate(rot);

        // 5. Slow mode (right bumper = precision mode)
        if (controller.leftTrigger().getAsBoolean()) {
            x   *= 0.35;
            y   *= 0.35;
            rot *= 0.45;
        }

        return drive.withVelocityX(x * drivetrain.maxSpd) // Drive forward with negative Y (forward)
            .withVelocityY(y * drivetrain.maxSpd) // Drive left with negative X (left)
            .withRotationalRate(rot * maxAngularRate); // Drive counterclockwise with negative X (left)
    }

    private void applyStates() {
        switch (wantedState) {
            case MANUAL_DRIVING:
                SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
                drivetrain.setSwerveState(GetSwerverCommand(drive));
                break;
            case AUTOMATIC_DRIVING:
                // TODO: something here, idk
                break;
            case IDLING:
                break;
            default:
                drivetrain.setSwerveState(new SwerveRequest.Idle());
                break;
        }

        if (wantedState != previousWantedState) {
            GreenLogger.log("Swerve state: " + wantedState.toString());
            SmartDashboard.putString("Swerve state: ", wantedState.toString());
            previousWantedState = wantedState;
        }
    }

    public void setWantedState(ActualState state) {
        this.wantedState = state;
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position
     *                                 in meters, y position in meters, and heading
     *                                 in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs)
    {
        drivetrain.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs
        );
    }

    /**
     * Sets the pose estimator's trust in robot odometry. This might be used to change trust in
     * odometry after an impact with the wall or traversing a bump.
     *
     * @param stateStdDevs Standard deviations of the pose estimate. Increase these numbers to
     *                     trust your state estimate less. This matrix is in the form [x, y,
     *                     theta]ᵀ, with units in meters and radians.
     */
    public void setStateStdDevs(Matrix<N3, N1> stateStdDevs) {
        drivetrain.setStateStdDevs(stateStdDevs);
    }

    public void simTeleportRobot(Pose2d pose) {
        drivetrain.simTeleportRobot(pose);
    }
}
