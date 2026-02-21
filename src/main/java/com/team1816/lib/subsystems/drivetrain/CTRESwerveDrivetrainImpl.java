package com.team1816.lib.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.team1816.lib.BaseRobotState;
import com.team1816.lib.hardware.SubsystemConfig;
import com.team1816.lib.hardware.components.motor.WpiMotorUtil;
import com.team1816.lib.util.GreenLogger;
import com.team1816.lib.util.SwerveDriveStateStruct;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;


import static com.team1816.lib.Singleton.factory;
import static com.team1816.lib.util.FormatUtils.GetDisplay;

public class CTRESwerveDrivetrainImpl extends SwerveDrivetrain<CommonTalon, CommonTalon, ParentDevice> implements IDrivetrain {

    SubsystemConfig config = factory.getSubsystemConfig(NAME);
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private double lastSimTime;
    private boolean fieldCentric;
    /**
     * An odometry object representing the "actual" position of the robot in the sim, or the raw
     * odometry (without vision estimates) on the real robot. These are the same thing because the
     * sim actual position is intended to be used by vision as the place the cameras simulate
     * seeing from to create simulated camera frames, meaning that it should be a pre-vision
     * position used as the source of truth during simulation, while the raw odometry position is
     * just supposed to be the pre-vision position estimate of the real robot.
     */
    private final SwerveDriveOdometry simActualOdometryOrRawOdometry;

    /**
     * Swerve request to apply during robot-centric path following
     */
    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    // Creates the CTRE swerve drivetrain.  The getDeviceById calls are made by the CTRE class and are based
    // on the defined values in the getSwerveModuleConstants
    public CTRESwerveDrivetrainImpl() {
        super((id, bus) -> (CommonTalon) factory.getDeviceById(NAME, id),
            (id, bus) -> (CommonTalon) factory.getDeviceById(NAME, id),
            (id, bus) -> (ParentDevice) factory.getDeviceById(NAME, id),
            factory.getSwerveDrivetrainConstant(NAME),
            // Pass in 0 as the odometryUpdateFrequency to let CTRE just use their defaults.
            0,
            // The initial odometryStandardDeviations to use until a call to setStateStdDevs. I
            // don't know why CTRE calls them to different things, but I'm pretty sure the state
            // and odometry standard deviations are the same. They represent the pose estimator's
            // trust in the current state of the odometry estimate, with higher values representing
            // less trust.
            VecBuilder.fill(0.1, 0.1, 0.1),
            // The initial visionStandardDeviations used here will be overwritten any time
            // addVisionMeasurement is called with visionMeasurementStdDevs, so these initial
            // values don't actually matter unless we decide not to pass them in with
            // addVisionMeasurement. They represent the pose estimator's trust in the incoming
            // vision measurements, with higher values representing less trust.
            VecBuilder.fill(1, 1, 1),
            factory.getSwerveModuleConstants(NAME, maxSpd)
        );
        //default to filed centric
        fieldCentric = factory.getConstant(NAME, "fieldCentric", 1) == 1;
        GreenLogger.log("FieldCentric: " + fieldCentric);

        configureAutoBuilder();

        simActualOdometryOrRawOdometry = new SwerveDriveOdometry(
            getKinematics(), getState().RawHeading, getState().ModulePositions
        );

        // Register a lambda to be updated whenever CTRE's odometry thread runs and updates the
        // SwerveDriveState. This updates the robotPose and robotSpeeds in the BaseRobotState
        // and updates the sim actual odometry/raw odometry to make it is always a vision-less
        // version of the SwerveDrivetrain's pose estimate.
        registerTelemetry(state -> {
            BaseRobotState.robotPose = state.Pose;
            BaseRobotState.robotSpeeds = state.Speeds;

            simActualOdometryOrRawOdometry.update(state.RawHeading, state.ModulePositions);
            BaseRobotState.simActualOrRawOdometryPose = simActualOdometryOrRawOdometry
                .getPoseMeters();
        });

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void setSwerveState(SwerveRequest request) {
        this.setControl(request);
    }

    @Override
    public void resetPose(Pose2d pose) {
        simActualOdometryOrRawOdometry.resetPose(pose);
        super.resetPose(pose);
    }

    @Override
    public void simTeleportRobot(Pose2d pose) {
        if (Utils.isSimulation()) {
            simActualOdometryOrRawOdometry.resetPose(pose);
        }
        else {
            GreenLogger.log(
                "Unfortunately, the robot cannot teleport in real life. We are still working on " +
                    "that feature."
            );
        }
    }

    @Override
    public void setUpPeriodicLogging(String logPath) {
        GreenLogger.periodicLog(
            logPath + "Swerve Drive State",
            this::getStateCopy,
            new SwerveDriveStateStruct(getModules().length)
        );
        GreenLogger.periodicLog(
            logPath + "Sim Actual or Raw Odometry Pose",
            () -> BaseRobotState.simActualOrRawOdometryPose,
            Pose2d.struct
        );
    }

    @Override
    public boolean IsFieldCentric() {
        return fieldCentric;
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    private void configureAutoBuilder() {

        var drConf = config.devices.get("flDr");
        var driveMotor = WpiMotorUtil.getMotorConstants(drConf).withReduction(gearing);
        var moduleConfig = new ModuleConfig(whlRad, maxSpd, cof, driveMotor, factory.GetCurrentConfigs(drConf).StatorCurrentLimit, 1);
        // In order of front left, front right, back left, back right
        var moduleLocations = this.getModuleLocations();
        RobotConfig robotConfig;
        PathFollowingController pathFollowingController;

        robotConfig = new RobotConfig(massKG, MOI, moduleConfig, moduleLocations);
        var tranKp = factory.getConstant(NAME, "translationKp", 5);
        var rotKp = factory.getConstant(NAME, "rotationKp", 5);
        GreenLogger.log(
            "translationKp:" + GetDisplay(tranKp) +
                " rotationKp:" + GetDisplay(rotKp)
        );
        pathFollowingController = new PPHolonomicDriveController(
            // PID constants for translation
            new PIDConstants(tranKp, 0, 0),
            // PID constants for rotation
            new PIDConstants(rotKp, 0, 0),
            // Period
            .02
        );

        GreenLogger.log(robotConfig);
        AutoBuilder.configure(
            () -> getState().Pose,   // Supplier of current robot pose
            (pose) -> { // Consumer for seeding pose against auto
                GreenLogger.log("Setting " + pose);
                resetPose(pose);
            },
            () -> getState().Speeds, // Supplier of current robot speeds
            // Consumer of ChassisSpeeds and feedforwards to drive the robot
            (speeds, feedforwards) -> setControl(
                pathApplyRobotSpeeds.withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ),
            pathFollowingController,
            robotConfig,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Subsystem for requirements
        );
    }

    /**
     * Adds vision measurements to CTRE's SwerveDrivetrain to combine with odometry data to
     * estimate the drivetrain pose. This method takes in an FGPA timestamp for the vision estimate
     * (which is returned by PhotonVision) and converts it to a timestamp in CTRE's expected time
     * base before passing off the parameters to CTRE's addVisionMeasurement.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in seconds. This
     *                                 should be an FGPA timestamp, which is what PhotonVision
     *                                 returns as part of its {@link
     *                                 org.photonvision.EstimatedRobotPose}.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x
     *                                 position in meters, y position in meters, and heading in
     *                                 radians). Increase these numbers to trust the vision pose
     *                                 measurement less.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs)
    {
        super.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(timestampSeconds),
            visionMeasurementStdDevs
        );
    }
}
