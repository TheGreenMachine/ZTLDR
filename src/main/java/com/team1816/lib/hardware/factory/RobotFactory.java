package com.team1816.lib.hardware.factory;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.path.PathConstraints;
import com.team1816.lib.commands.PathfindToPoseCommand;
import com.team1816.lib.hardware.*;
import com.team1816.lib.hardware.components.GhostDevice;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.lib.hardware.components.gyro.Pigeon2Impl;
import com.team1816.lib.hardware.components.led.CANdleImpl;
import com.team1816.lib.hardware.components.led.CANifierImpl;
import com.team1816.lib.hardware.components.motor.TalonFXImpl;
import com.team1816.lib.hardware.components.motor.TalonFXSImpl;
import com.team1816.lib.hardware.components.sensor.CANCoderImpl;
import com.team1816.lib.hardware.components.sensor.CANdiImpl;
import com.team1816.lib.hardware.components.sensor.CanRangeImpl;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.*;

import static com.team1816.lib.Singleton.factory;
import static com.team1816.lib.hardware.components.motor.WpiMotorUtil.getModuleName;
import static com.team1816.lib.util.FormatUtils.GetDisplay;


public class RobotFactory {

    private RobotConfiguration config;
    public boolean robotIsReal; // Use to detect real or simulation public to override for tests
    private static final int startingGhostId = 50;
    private final Map<String, CANBus> canBusMap = new HashMap<>();

    /**
     * Initializes robot configuration by using ROBOT_NAME environment variable to read the correct YAML.
     * <p>
     * Does not throw any exceptions on error loading the config.
     */
    public RobotFactory() {
        robotIsReal = RobotBase.isReal();
        var robotName = System.getenv("ROBOT_NAME");
        if (robotName == null) {
            GreenLogger.log("ROBOT_NAME environment variable not defined using test yaml");
            robotName = "test";
        }
        robotName = robotName.toLowerCase();
        GreenLogger.log("Loading Config for " + robotName);
        try {
            config = YamlConfig.loadFrom(
                this.getClass().getClassLoader().getResourceAsStream("yaml/" + robotName + ".yml")
            );
        } catch (Exception e) {
            GreenLogger.log("Yaml Config error!" + e.getMessage());
        }
    }

    public Map<String, Double> getConstants() {
        return config.constants;
    }

    public SubsystemConfig getSubsystemConfig(String subsystemName) {
        var subsystem = config.subsystems.get(subsystemName);
        if (subsystem == null) {
            subsystem = new SubsystemConfig();
            subsystem.implemented = false;
        }
        subsystem.name = subsystemName;
        return subsystem;
    }

    public double getConstant(String name, double defaultVal) {
        if (getConstants() == null || !getConstants().containsKey(name)) {
            DriverStation.reportWarning("Yaml constants: " + name + " missing", true);
            return defaultVal;
        }
        return getConstants().get(name);
    }

    public double getConstant(String subsystemName, String name, double defaultVal) {
        return getConstant(subsystemName, name, defaultVal, defaultVal == 0);
    }

    public double getConstant(
        String subsystemName,
        String name,
        double defaultVal,
        boolean showWarning
    ) {
        if (config == null || !getSubsystemConfig(subsystemName).implemented) {
            return defaultVal;
        }
        if (
            getSubsystemConfig(subsystemName).constants == null ||
                !getSubsystemConfig(subsystemName).constants.containsKey(name)
        ) {
            if (showWarning) {
                DriverStation.reportWarning(
                    "Yaml: subsystem \"" +
                        subsystemName +
                        "\" constant \"" +
                        name +
                        "\" missing",
                    defaultVal == 0
                );
            }
            return defaultVal;
        }
        return getSubsystemConfig(subsystemName).constants.get(name);
    }

    public ShooterSettingsConfig getShooterSettingsConfig() {
        if (config.shooterSettings == null) {
            GreenLogger.log("Couldn't find the shooter settings config");
            return new ShooterSettingsConfig();
        }
        return config.shooterSettings;
    }

    /**
     * Retrieves the name of the default auto.
     */
    public String getDefaultAuto() {
        return config.defaultAuto;
    }

    /**
     * Retrieves the names of all the paths.
     */
    public Set<String> getPathNames() {
        return config.autopathing.paths.keySet();
    }

    /**
     * Gets the {@link PathConfig} under a single path name.
     */
    public PathConfig getPathConfig(String pathName) {
        PathConfig pathConfig = config.autopathing.paths.get(pathName);
        if (pathConfig.constraints == null) {
            pathConfig.constraints = config.autopathing.constraints;
        }

        return pathConfig;
    }

    /**
     * Retrieves all the {@link PathfindToPoseCommand PathfindToPoseCommands} in configuration. Automatically populates defaults.
     */
    public List<PathfindToPoseCommand> getPaths() {
        List<PathfindToPoseCommand> paths = new ArrayList<>(factory.getPathNames().size());

        for (String pathName : factory.getPathNames()) {
            PathConfig config = factory.getPathConfig(pathName);

            PathConstraintsConfig constraintsConfig = config.constraints;

            PathConstraints constraints = new PathConstraints(
                constraintsConfig.maxVelocity,
                constraintsConfig.maxAccel,
                Units.degreesToRadians(constraintsConfig.maxAngularVelocity),
                Units.degreesToRadians(constraintsConfig.maxAngularAccel)
            );

            PathfindToPoseCommand command = new PathfindToPoseCommand(
                pathName,
                new Pose2d(
                    new Translation2d(config.x, config.y),
                    Rotation2d.fromDegrees(config.rotation)
                ),
                constraints,
                config.flippable,
                config.targetVelocity
            );

            paths.add(command);
        }

        return Collections.unmodifiableList(paths);
    }

    /**
     * Retrieves a {@link IPhoenix6 device} for a specific subsystem under a name.
     */
    public IPhoenix6 getDevice(String subsystemName, String deviceName) {
        if (config == null) {
            throw new NullPointerException("config is null");
        }
        var subsystem = getSubsystemConfig(subsystemName);

        // we should have devices defined verify that
        if (subsystem.devices == null) {
            throw new IllegalArgumentException("Devices not defined for " + subsystemName);
        }
        // ensure device is defined in yaml
        var deviceConfig = subsystem.devices.get(deviceName);

        if (deviceConfig == null) {
            throw new IllegalArgumentException("Device " + deviceName + " not found");
        }
        deviceConfig.name = deviceName;
        // ensure device type is set
        if (deviceConfig.deviceType == null) {
            throw new IllegalArgumentException("deviceType missing in yml");
        }
        return getDevInst(deviceConfig, subsystem, true);
    }

    /**
     * Used to get devices by their ID. Used by CTRE swerve.
     */
    public IPhoenix6 getDeviceById(String subsystemName, int id) {
        var config = factory.getSubsystemConfig(subsystemName);
        // CTRE swerve cannot use ghosted devices they look for specific types
        // and will die when created, so send nulls instead
        if (!config.implemented) return null;
        for (var key : config.devices.keySet()) {
            var device = config.devices.get(key);
            if (device.id == id) {
                // ensure the name is set
                if (device.name == null) device.name = key;
                return getDevInst(device, config, true);
            }
        }
        return null;
    }

    /**
     * Instantiates device based on the type and applies the configuration
     */
    private IPhoenix6 getDevInst(DeviceConfiguration deviceConfig, SubsystemConfig subsystemConfig, boolean logDetails) {
        IPhoenix6 devInst = null;
        var bus = subsystemConfig.canBusName;
        if (!subsystemConfig.implemented || deviceConfig.id > startingGhostId) {
            GreenLogger.log("Device " + deviceConfig.name + " not implemented ghosting");
            bus = "ghost";
        } else {
            GreenLogger.log("Creating " + deviceConfig.name);
        }
        GreenLogger.log("  id: " + deviceConfig.id);
        GreenLogger.log("  deviceType: " + deviceConfig.deviceType);

        CANBus canbus;
        if (!canBusMap.containsKey(bus)) {
            canbus = new CANBus(bus);
            canBusMap.put(bus, canbus);
        } else {
            canbus = canBusMap.get(bus);
        }

        if (!subsystemConfig.implemented) return new GhostDevice(deviceConfig.id, canbus);

        switch (deviceConfig.deviceType) {
            case TalonFX  -> devInst = new TalonFXImpl(deviceConfig.id, canbus);
            case TalonFXS -> devInst = new TalonFXSImpl(deviceConfig.id, canbus);
            case Pigeon2  -> devInst = new Pigeon2Impl(deviceConfig.id, canbus);
            case CANdle   -> devInst = new CANdleImpl(deviceConfig.id, canbus);
            case CANdi    -> devInst = new CANdiImpl(deviceConfig.id, canbus);
            case CANrange -> devInst = new CanRangeImpl(deviceConfig.id, canbus);
            case CANifier -> devInst = new CANifierImpl(deviceConfig.id);
            case CANcoder -> devInst = new CANCoderImpl(deviceConfig.id, canbus);
            default -> GreenLogger.log("Device type " + deviceConfig.deviceType + " not implemented");
        }
        var parentConfig = getCTREConfig(subsystemConfig, deviceConfig);
        if (parentConfig != null) {
            var logPath = subsystemConfig.name + "/" + deviceConfig.name + "/";
            // apply the configuration
            devInst.applyConfiguration(parentConfig, logPath, logDetails);
        }
        return devInst;
    }

    /**
     * Takes YAML Device configuration and creates a CTRE configuration object
     */
    private ParentConfiguration getCTREConfig(SubsystemConfig subsystemConfig, DeviceConfiguration deviceConfig) {
        ParentConfiguration parentConfig = null;
        switch (deviceConfig.deviceType) {
            case CANifier -> {
                // this is phoenix 5 there is no config
            }
            case TalonFX  -> parentConfig = new TalonFXConfiguration();
            case TalonFXS -> parentConfig = new TalonFXSConfiguration();
            case Pigeon2  -> parentConfig = new Pigeon2Configuration();
            case CANdle   -> parentConfig = new CANdleConfiguration();
            case CANdi    -> parentConfig = new CANdiConfiguration();
            case CANrange -> parentConfig = new CANrangeConfiguration();
            case CANcoder -> parentConfig = new CANcoderConfiguration();
            default -> GreenLogger.log("Unknown CTRE configuration for deviceType: " + deviceConfig.deviceType);
        }
        if (parentConfig != null) {
            // Update defaults with yaml values
            applyYamlConfigs(subsystemConfig, deviceConfig, parentConfig);
        }
        return parentConfig;
    }

    private void applyYamlConfigs(SubsystemConfig subsystemConfig, DeviceConfiguration deviceConfig, ParentConfiguration parentConfig) {
        switch (deviceConfig.deviceType) {
            case TalonFX -> {
                var clazz = (TalonFXConfiguration) parentConfig;
                clazz.MotorOutput = getMotorOutputConfigs(deviceConfig);
                clazz.Slot0 = Slot0Configs.from(getSlotConfigs(deviceConfig.pidConfig, 0));
                clazz.Slot1 = Slot1Configs.from(getSlotConfigs(deviceConfig.pidConfig, 1));
                clazz.Slot2 = Slot2Configs.from(getSlotConfigs(deviceConfig.pidConfig, 2));
                clazz.CurrentLimits = getCurrentConfigs(deviceConfig);
                clazz.SoftwareLimitSwitch = getSoftLimitConfigs(deviceConfig);
                clazz.Feedback = GetFeedbackConfigs(deviceConfig);
                clazz.MotionMagic = GetMotionMagicConfig(deviceConfig);
            }
            case TalonFXS -> {
                var clazz = (TalonFXSConfiguration) parentConfig;
                clazz.MotorOutput = getMotorOutputConfigs(deviceConfig);
                clazz.Commutation = getCommunicationConfigs(deviceConfig);
                clazz.Slot0 = Slot0Configs.from(getSlotConfigs(deviceConfig.pidConfig, 0));
                clazz.Slot1 = Slot1Configs.from(getSlotConfigs(deviceConfig.pidConfig, 1));
                clazz.Slot2 = Slot2Configs.from(getSlotConfigs(deviceConfig.pidConfig, 2));
                clazz.CurrentLimits = getCurrentConfigs(deviceConfig);
                clazz.SoftwareLimitSwitch = getSoftLimitConfigs(deviceConfig);
                clazz.ExternalFeedback = getExternalFeedbackConfigs(deviceConfig);
                clazz.MotionMagic = GetMotionMagicConfig(deviceConfig);
            }
            case Pigeon2 -> {
            }
            case CANdle -> {
                var clazz = (CANdleConfiguration) parentConfig;
                clazz.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
                clazz.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
                clazz.LED.StripType = StripTypeValue.BRG;
            }
            case CANdi -> {
                var clazz = (CANdiConfiguration) parentConfig;
            }
            case CANifier -> {
            }
            case CANrange -> {
                var clazz = (CANrangeConfiguration) parentConfig;
            }
        }
    }

    /**
     * Retrieves {@link FeedbackConfigs} from a particular {@link DeviceConfiguration}. Logs these values.
     */
    private MotionMagicConfigs GetMotionMagicConfig(DeviceConfiguration deviceConfig) {
        var mMConfig = new MotionMagicConfigs();
        if (deviceConfig.motionMagic == null) {
            return mMConfig;
        }

        mMConfig.MotionMagicExpo_kA = deviceConfig.motionMagic.expoKA;
        mMConfig.MotionMagicExpo_kV = deviceConfig.motionMagic.expoKV;
        return mMConfig;
    }

    private FeedbackConfigs GetFeedbackConfigs(DeviceConfiguration deviceConfig) {
        var config = new FeedbackConfigs();
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.remoteSensor != null) {
            switch (deviceConfig.remoteSensor) {
                case RemoteCANcoder -> config.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            }
        }
        GreenLogger.log("  remoteSensor: " + config.FeedbackSensorSource);

        if (deviceConfig.remoteSensorId != null) {
            config.FeedbackRemoteSensorID = deviceConfig.remoteSensorId;
            GreenLogger.log("  remoteSensorId: " + config.FeedbackRemoteSensorID);
        }

        if (deviceConfig.remoteOffest != null) {
            config.FeedbackRotorOffset = deviceConfig.remoteOffest;
            GreenLogger.log("  remoteOffest: " + config.FeedbackRotorOffset);
        }
        return config;
    }

    /**
     * Retrieves {@link ExternalFeedbackConfigs} from a particular {@link DeviceConfiguration}. Logs these values.
     */
    private ExternalFeedbackConfigs getExternalFeedbackConfigs(DeviceConfiguration deviceConfig) {
        var config = new ExternalFeedbackConfigs();
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.remoteSensor != null) {
            config.ExternalFeedbackSensorSource = deviceConfig.remoteSensor;
        }
        GreenLogger.log("  remoteSensor: " + config.ExternalFeedbackSensorSource);

        if (deviceConfig.remoteSensorId != null) {
            config.FeedbackRemoteSensorID = deviceConfig.remoteSensorId;
            GreenLogger.log("  remoteSensorId: " + config.FeedbackRemoteSensorID);
        }

        if (deviceConfig.remoteOffest != null) {
            config.AbsoluteSensorOffset = deviceConfig.remoteOffest;
            GreenLogger.log("  remoteOffest: " + config.AbsoluteSensorOffset);
        }

        return config;
    }

    /**
     * Retrieves {@link SoftwareLimitSwitchConfigs} from a particular {@link DeviceConfiguration}. Logs these values.
     */
    private SoftwareLimitSwitchConfigs getSoftLimitConfigs(DeviceConfiguration deviceConfig) {
        var config = new SoftwareLimitSwitchConfigs();
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.forwardSoftLimit != null) {
            config.ForwardSoftLimitThreshold = deviceConfig.forwardSoftLimit;
            config.ForwardSoftLimitEnable = true;
        }
        GreenLogger.log("  forwardSoftLimit: " + config.ForwardSoftLimitThreshold);
        if (deviceConfig.reverseSoftLimit != null) {
            config.ReverseSoftLimitThreshold = deviceConfig.reverseSoftLimit;
            config.ReverseSoftLimitEnable = true;
        }
        GreenLogger.log("  reverseSoftLimit: " + config.ReverseSoftLimitThreshold);

        return config;
    }

    /**
     * Retrieves {@link CurrentLimitsConfigs} from a particular {@link DeviceConfiguration}. Logs these values.
     */
    public CurrentLimitsConfigs getCurrentConfigs(DeviceConfiguration deviceConfig) {
        var config = new CurrentLimitsConfigs();

        // the defaults for current limits are on from CTRE we want to enforce this.
        // turning off current limits only breaks things
        config.StatorCurrentLimitEnable = true;
        config.SupplyCurrentLimitEnable = true;

        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.statorCurrentLimit != null) {
            config.StatorCurrentLimit = deviceConfig.statorCurrentLimit;
        }
        GreenLogger.log("  statorCurrentLimit: " + config.StatorCurrentLimit);

        if (deviceConfig.supplyCurrentLimit != null) {
            config.SupplyCurrentLimit = deviceConfig.supplyCurrentLimit;
        }
        GreenLogger.log("  supplyCurrentLimit: " + config.SupplyCurrentLimit);

        if (deviceConfig.supplyCurrentLowerLimit != null) {
            config.SupplyCurrentLowerLimit = deviceConfig.supplyCurrentLowerLimit;
        }
        GreenLogger.log("  supplyCurrentLowerLimit: " + config.SupplyCurrentLowerLimit);

        if (deviceConfig.supplyCurrentLowerTime != null) {
            config.SupplyCurrentLowerTime = deviceConfig.supplyCurrentLowerTime;
        }
        GreenLogger.log("  supplyCurrentLowerTime: " + config.SupplyCurrentLowerTime);

        return config;
    }

    /**
     * Slot configs are the PID values
     */
    private SlotConfigs getSlotConfigs(Map<String, PIDSlotConfiguration> pidConfig, int slot) {
        var config = new SlotConfigs();
        config.SlotNumber = slot;
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        var key = "slot" + slot;
        if (pidConfig != null && pidConfig.containsKey(key)) {
            var pid = pidConfig.get(key);
            if (pid.kA != null) config.kA = pid.kA;
            if (pid.kD != null) config.kD = pid.kD;
            if (pid.kG != null) config.kG = pid.kG;
            if (pid.kP != null) config.kP = pid.kP;
            if (pid.kS != null) config.kS = pid.kS;
            if (pid.kV != null) config.kV = pid.kV;
            if (pid.kI != null) config.kI = pid.kI;
            if (pid.gravityType != null) config.GravityType = pid.gravityType;
            GreenLogger.log("  " + key +
                " - kP:" + GetDisplay(config.kP) +
                " kI:" + GetDisplay(config.kI) +
                " kD:" + GetDisplay(config.kD) +
                " kV:" + GetDisplay(config.kV) +
                " kS:" + GetDisplay(config.kS) +
                " kA:" + GetDisplay(config.kA) +
                " kG:" + GetDisplay(config.kG) +
                " gravityType:" + config.GravityType);
        }
        return config;
    }

    /**
     * The Communication configs are used to configure motor type and connections
     */
    private CommutationConfigs getCommunicationConfigs(DeviceConfiguration deviceConfig) {
        var config = new CommutationConfigs();
        String info = "";
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.motorType != null) {
            config.MotorArrangement = deviceConfig.motorType;
            if (deviceConfig.motorType == MotorArrangementValue.Brushed_DC) {
                config.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_C;
                info = " using Leads_A_and_C";
            }
        } else {
            throw new IllegalArgumentException("motorType config is missing in yml for " + deviceConfig.name);
        }
        GreenLogger.log("  motorType: " + config.MotorArrangement + info);
        return config;
    }

    /**
     * These configurations control the motor inversions and neutral behaviour
     */
    private MotorOutputConfigs getMotorOutputConfigs(DeviceConfiguration deviceConfig) {
        var config = new MotorOutputConfigs();
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.motorRotation != null) {
            config.Inverted = deviceConfig.motorRotation;
        }
        GreenLogger.log("  motorRotation: " + config.Inverted);
        if (deviceConfig.neutralMode != null) {
            config.NeutralMode = deviceConfig.neutralMode;
        }
        GreenLogger.log("  neutralMode: " + config.NeutralMode);
        return config;
    }

    /**
     * Gets CTRE Swerve Modules
     *
     * @see <a href="https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/mechanisms/swerve/swerve-builder-api.html">Swerve Builder</a>
     */
    public SwerveDrivetrainConstants getSwerveDrivetrainConstant(String subsystemName) {
        var config = getSubsystemConfig(subsystemName);
        var constants = new SwerveDrivetrainConstants();
        constants.CANBusName = config.canBusName;
        constants.Pigeon2Id = config.devices.get("gyro").id;
        return constants;
    }

    public SwerveModuleConstants<?, ?, ?>[] getSwerveModuleConstants(String subsystemName, double maxSpd) {
        var config = getSubsystemConfig(subsystemName);
        var kinematics = config.kinematics;
        verifyKinematics(subsystemName, kinematics);
        var constants = new SwerveModuleConstants[config.modules.size()];
        var x = config.kinematics.wheelbaseWidth / 2;
        var y = config.kinematics.wheelbaseLength / 2;
        int i = 0;
        ParentConfiguration driveConf = null;
        ParentConfiguration steerConf = null;
        Slot0Configs driveSlot = null;
        Slot0Configs steerSlot = null;

        // use ctre swerve factory to create the constants
        var factory = new SwerveModuleConstantsFactory();

        // setup kinematics
        factory.WheelRadius = kinematics.wheelRadius;
        factory.DriveMotorGearRatio = kinematics.driveGearing;
        factory.SteerMotorGearRatio = kinematics.steerGearing;
        factory.SpeedAt12Volts = maxSpd;

        for (var module : config.modules.values()) {
            var drive = config.devices.get(module.drive);
            var azm = config.devices.get(module.azimuth);
            // pull module configs from the first module
            if (driveConf == null) {
                GreenLogger.log("Module Drive Configuration");
                driveConf = getCTREConfig(config, drive);
                driveSlot = Slot0Configs.from(getSlotConfigs(config.drivePID, 0));
                GreenLogger.log("Module Azimuth Configuration");
                steerConf = getCTREConfig(config, azm);
                steerSlot = Slot0Configs.from(getSlotConfigs(config.azimuthPID, 0));
                GreenLogger.log("Creating " + config.modules.size() + " Modules");
                GreenLogger.log(" WheelRadius: " + factory.WheelRadius);
                GreenLogger.log(" DriveMotorGearRatio: " + factory.DriveMotorGearRatio);
                GreenLogger.log(" SteerMotorGearRatio: " + factory.SteerMotorGearRatio);
                GreenLogger.log(" SpeedAt12Volts: " + factory.SpeedAt12Volts);
            }

            // CTRE overwrites some of the defaults see their docs
            factory.DriveMotorInitialConfigs = driveConf;
            factory.DriveMotorGains = driveSlot;
            factory.DriveMotorClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;

            factory.SteerMotorClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;
            factory.SteerMotorGains = steerSlot;
            factory.SteerMotorInitialConfigs = steerConf;
            factory.SteerMotorType = getSteerArrangement(azm.motorType);

            var constant = factory.createModuleConstants(
                azm.id,
                drive.id,
                azm.remoteSensorId == null ? azm.id : azm.remoteSensorId,
                azm.remoteOffest,
                i == 0 || i == 1 ? x : -x,
                i == 1 || i == 3 ? -y : y,
                drive.motorRotation == InvertedValue.Clockwise_Positive,
                azm.motorRotation == InvertedValue.Clockwise_Positive,
                false
            );
            GreenLogger.log("  " + getModuleName(i) + ": x:" + constant.LocationX + " y:" + constant.LocationY + " remoteOffest:" + azm.remoteOffest);
            constants[i] = constant;
            i++;
        }
        return constants;
    }

    /**
     * Logs missing values to kinematics yaml configuration
     */
    private void verifyKinematics(String subsystemName, KinematicsConfig kinematics) {
        if (kinematics == null) {
            var message = subsystemName + " requires kinematics in yaml";
            throw new NullPointerException(message);
        }
        String property = null;
        if (kinematics.driveGearing == 0) {
            property = "driveGearing";
        }
        if (kinematics.steerGearing == 0) {
            property = "steerGearing";
        }
        if (kinematics.maxDriveRPS == 0) {
            property = "maxDriveRPS";
        }
        if (kinematics.wheelbaseLength == 0) {
            property = "wheelbaseLength";
        }
        if (kinematics.wheelbaseWidth == 0) {
            property = "wheelbaseWidth";
        }
        if (kinematics.robotMass == 0) {
            property = "robotMass";
        }
        if (kinematics.wheelCOF == 0) {
            property = "wheelCOF";
        }
        if (kinematics.wheelRadius == 0) {
            property = "wheelRadius";
        }
        if (kinematics.maxAngularRate == null || kinematics.maxAngularRate == 0) {
            property = "maxAngularRate";
        }
        if (property != null) {
            var message = subsystemName + " kinematics " + property + " can't be null or 0";
            throw new IllegalArgumentException(message);
        }
    }

    private SwerveModuleConstants.SteerMotorArrangement getSteerArrangement(MotorArrangementValue value) {
        var steer = SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated;
        if (value == MotorArrangementValue.Brushed_DC) {
            return SwerveModuleConstants.SteerMotorArrangement.TalonFXS_Brushed_AC;
        } else if (value == MotorArrangementValue.Minion_JST) {
            return SwerveModuleConstants.SteerMotorArrangement.TalonFXS_Minion_JST;
        }
        return steer;
    }
}
