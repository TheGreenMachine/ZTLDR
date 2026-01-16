package com.team1816.lib.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.SubsystemDataProcessor;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Swerve extends SubsystemBase implements SubsystemDataProcessor.IDataRefresher, ITestableSubsystem {
    private IDrivetrain drivetrain;

    private CommandXboxController controller;
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);    // forward/back
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);    // strafe
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);  // rotation
    private static double maxAngularRate = 0;

    private SWERVE_STATE wantedState = SWERVE_STATE.SWERVE_IDLE;

    public Swerve(IDrivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        var kinematics = drivetrain.getKinematicsConfig();
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // setup teleop drivetrain command
        maxAngularRate = RotationsPerSecond.of(kinematics.maxAngularRate).in(RadiansPerSecond);

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(this);
    }

    public enum SWERVE_STATE {
        SWERVE_DRIVE,
        SWERVE_IDLE
    }

    @Override
    public void periodic() {
        applyStates();
    }

    @Override
    public void readFromHardware() {}

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
        if (controller.rightBumper().getAsBoolean()) {
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
            case SWERVE_DRIVE:
                SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
                drivetrain.setSwerveState(GetSwerverCommand(drive));
                break;
            case SWERVE_IDLE:
            default:
                drivetrain.setSwerveState(new SwerveRequest.Idle());
                break;
        }
    }

    public void setWantedState(SWERVE_STATE state) {
        this.wantedState = state;
    }
}
