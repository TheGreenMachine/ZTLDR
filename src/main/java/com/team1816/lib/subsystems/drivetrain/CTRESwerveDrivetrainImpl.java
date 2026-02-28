package com.team1816.lib.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
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
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

import static com.team1816.lib.BaseConstants.DrivetrainConstants;
import static com.team1816.lib.Singleton.factory;
import static com.team1816.lib.util.FormatUtils.GetDisplay;

public class CTRESwerveDrivetrainImpl extends SwerveDrivetrain<CommonTalon, CommonTalon, ParentDevice> implements IDrivetrain {

    SubsystemConfig config = factory.getSubsystemConfig(NAME);
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private double lastSimTime;
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
     * The latest state standard deviations sent to the pose estimator, for logging purposes.
     */
    private Matrix<N3, N1> latestStateStdDevs = DrivetrainConstants.defaultStateStdDevs;

    /**
     * Swerve request to apply during robot-centric path following
     */
    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    // Creates the CTRE swerve drivetrain.  The getDeviceById calls are made by the CTRE class and are based
    // on the defined values in the getSwerveModuleConstants
    public CTRESwerveDrivetrainImpl(
        DeviceConstructor<CommonTalon> driveMotorConstructor,
        DeviceConstructor<CommonTalon> steerMotorConstructor,
        DeviceConstructor<ParentDevice> encoderConstructor,
        SwerveDrivetrainConstants drivetrainConstants
    ) {
        super(
            driveMotorConstructor,
            steerMotorConstructor,
            encoderConstructor,
            drivetrainConstants,
            // Pass in 0 as the odometryUpdateFrequency to let CTRE just use their defaults.
            0,
            // The initial odometryStandardDeviations to use until a call to setStateStdDevs. I
            // don't know why CTRE calls them two different things, but I'm pretty sure the state
            // and odometry standard deviations are the same. They represent the pose estimator's
            // trust in the current state of the odometry estimate, with higher values representing
            // less trust.
            DrivetrainConstants.defaultStateStdDevs,
            // The initial visionStandardDeviations used here will be overwritten any time
            // addVisionMeasurement is called with visionMeasurementStdDevs, so these initial
            // values don't actually matter unless we decide not to pass them in with
            // addVisionMeasurement. They represent the pose estimator's trust in the incoming
            // vision measurements, with higher values representing less trust.
            VecBuilder.fill(1, 1, 1),
            factory.getSwerveModuleConstants(NAME, maxSpd)
        );

        configureAutoBuilder();

        simActualOdometryOrRawOdometry = new SwerveDriveOdometry(
            getKinematics(), getState().RawHeading, getState().ModulePositions
        );

        // Register a lambda to be updated whenever CTRE's odometry thread runs and updates the
        // SwerveDriveState. This updates the BaseRobotState and updates the sim actual odometry/
        // raw odometry to make it is always a vision-less version of the SwerveDrivetrain's pose
        // estimate.
        registerTelemetry(state -> {
            BaseRobotState.robotPose = state.Pose;
            BaseRobotState.robotSpeeds = state.Speeds;
            BaseRobotState.robotTiltRadians = getTiltRadians();

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
    public void setStateStdDevs(Matrix<N3, N1> stateStdDevs) {
        latestStateStdDevs = stateStdDevs;
        super.setStateStdDevs(stateStdDevs);
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
        GreenLogger.periodicLog(
            logPath + "Robot Tilt (radians)",
            () -> BaseRobotState.robotTiltRadians
        );
        GreenLogger.periodicLog(
            logPath + "Latest State Standard Deviations",
            () -> latestStateStdDevs,
            Matrix.getStruct(Nat.N3(), Nat.N1())
        );
        GreenLogger.periodicLog(
            logPath + "Has Accurate Pose Estimate",
            () -> BaseRobotState.hasAccuratePoseEstimate
        );
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
        var moduleConfig = new ModuleConfig(whlRad, maxSpd, cof, driveMotor, factory.getCurrentConfigs(drConf).StatorCurrentLimit, 1);
        // In order of front left, front right, back left, back right
        var moduleLocations = this.getModuleLocations();
        RobotConfig robotConfig;
        PathFollowingController pathFollowingController;

        robotConfig = new RobotConfig(massKG, MOI, moduleConfig, moduleLocations);
        var tranKp = factory.getConstant(NAME, "translationKp", 5.0);
        //var tranKd = factory.getConstant(NAME, "translationKd", 0.2);
        var rotKp = factory.getConstant(NAME, "rotationKp", 5.0);
        var rotKi = factory.getConstant(NAME, "rotationKi", 0);
        var rotKd = factory.getConstant(NAME, "rotationKd", 0);
        GreenLogger.log(
            "translationKp:" + GetDisplay(tranKp) +
                " rotationKp:" + GetDisplay(rotKp)
        );
        var transPID = new PIDConstants(tranKp, 0, 0);
        var rotPID = new PIDConstants(rotKp, rotKi, rotKd);
        GreenLogger.log("transPID", transPID);
        GreenLogger.log("rotPID", rotPID);
        pathFollowingController = new PPHolonomicDriveController(
            // PID constants for translation
            transPID,
            // PID constants for rotation
            rotPID,
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

    /**
     * Gets the tilt of the robot relative to the field, in radians.
     *
     * @return The tilt of the robot relative to the field, in radians.
     */
    private double getTiltRadians() {
        // This method is ultimately trying to find the angle between a vector pointing up in the
        // field reference frame and a vector pointing up in the robot reference frame, as this
        // will tell us how tilted the robot is relative to the field. The vector pointing up in
        // the robot reference frame means the same thing as a vector pointing along the robot's
        // z-axis.

        // We start by getting the Rotation3d of the robot from CTRE.
        Rotation3d rotation = getRotation3d();

        // The toMatrix method on the Rotation3d gets a rotation matrix representation of the
        // rotation. This represents the unit vectors for the x, y, and z axes, in the field
        // reference frame. For instance, the first column in the matrix represents the unit vector
        // of the x-axis in the field coordinate frame, with the value in the first row and first
        // column being the x component of this x-axis vector.
        Matrix<N3, N3> rotationMatrix = rotation.toMatrix();

        // By calling the zero-indexed method get(2, 2), we get the z component of the z-axis unit
        // vector.
        double zComponent = rotationMatrix.get(2, 2);

        // A bit of trigonometry tells us that our angle will be the inverse cosine of the z
        // component of the robot's z-axis.
        //
        //                 |\ <- theta = Angle between the robot z-axis and the field z-axis
        //                 | \
        //                 |  \         cos(theta) = a/h = z component/1 = z component
        //                 |   \        theta = arccos(z component)
        //                 |    \
        // a = z component |     \ h = Robot z-axis unit vector = 1 (because it's a unit vector)
        //                 |      \
        //                 |_______\
        //
        return Math.acos(zComponent);
    }
}
