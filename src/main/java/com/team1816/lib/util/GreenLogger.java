package com.team1816.lib.util;

import com.pathplanner.lib.config.RobotConfig;
import com.team1816.season.Robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

public class GreenLogger {

    @SuppressWarnings("rawtypes")
    private static final HashMap<LogTopic, Supplier> periodicLogs = new HashMap<>();
    // using an empty string here to make the logs and live views consistent
    private static final NetworkTable netTable;
    private static final StringPublisher msg;

    static {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        // this will log the robot modes i.e., auto enabled estop
        DriverStation.startDataLog(DataLogManager.getLog(), false);
        // Log network tables then we can use advantage scope on a live robot
        // and use the same layout for the logs
        DataLogManager.logNetworkTables(true);
        // don't log console since we output to network tables
        DataLogManager.logConsoleOutput(false);
        netTable = NetworkTableInstance.getDefault().getTable("");
        msg = netTable.getStringTopic("messages").publish();
    }

    public static void SilenceLoopOverrun(Robot robot) {
        // Adjust loop overrun warning timeout
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(robot);
            watchdog.setTimeout(.2);
        } catch (Exception e) {
            log("Failed to disable loop overrun warnings.");
        }
        CommandScheduler.getInstance().setPeriod(.2);
    }

    /**
     * Adds a new periodic log.
     * <p>
     * This supports logging {@link Double}s, {@link Integer}s, {@link Boolean}s, {@link Float}s,
     * {@link String}s, {@link StructSerializable} classes (if the {@link Struct} is passed in, see
     * below), and {@link NTSendable}s. Anything else passed in will be logged as a {@link String}.
     * <p>
     * If you are logging something that is {@link StructSerializable} (for example, a
     * {@link Pose2d}, or various other WPILib classes), please use {@link #periodicLog(String,
     * Supplier, Struct)} to specify the {@link Struct} type.
     * <p>
     * If you are logging a {@link List}, please use {@link #periodicLogList(String, Supplier,
     * Class)} or {@link #periodicLogList(String, Supplier, Class, Struct)}.
     *
     * @param name The topic name to log under.
     * @param supplier A supplier of the values to log.
     * @param <T> The type returned by the supplier.
     */
    public static <T> void periodicLog(String name, Supplier<T> supplier) {
        periodicLog(name, supplier, null);
    }

    /**
     * Adds a new periodic log using the provided {@link Struct}.
     * <p>
     * Passing in a {@link Struct} is necessary to correctly log any {@link StructSerializable}
     * class (for example, a {@link Pose2d}, or various other WPILib classes). For most {@link
     * StructSerializable} classes, this can be obtained by getting the static {@code struct}
     * member variable from the class (for example, {@link Pose2d#struct}). If you are not logging
     * a {@link StructSerializable} class, you can leave out the {@link Struct} when calling this
     * method.
     * <p>
     * This supports logging {@link Double}s, {@link Integer}s, {@link Boolean}s, {@link Float}s,
     * {@link String}s, {@link StructSerializable} classes (if the {@link Struct} is passed in, see
     * above), and {@link NTSendable}s. Anything else passed in will be logged as a {@link String}.
     * <p>
     * If you are logging a {@link List}, please use {@link #periodicLogList(String, Supplier,
     * Class)} or {@link #periodicLogList(String, Supplier, Class, Struct)}.
     *
     * @param name The topic name to log under.
     * @param supplier A supplier of the values to log.
     * @param struct The {@link Struct} for the {@link StructSerializable} class being logged.
     * @param <T> The type returned by the supplier.
     */
    public static <T> void periodicLog(String name, Supplier<T> supplier, Struct<T> struct) {
        T result = supplier.get();
        Publisher pub;
        if (result instanceof Double) {
            pub = netTable.getDoubleTopic(name).publish();
        } else if (result instanceof Integer) {
            pub = netTable.getIntegerTopic(name).publish();
        } else if (result instanceof Boolean) {
            pub = netTable.getBooleanTopic(name).publish();
        } else if (result instanceof Float) {
            pub = netTable.getFloatTopic(name).publish();
        } else if (result instanceof StructSerializable && struct != null) {
            pub = netTable.getStructTopic(name, struct).publish();
        } else if (result instanceof NTSendable) {
            pub = null;
        } else {
            pub = netTable.getStringTopic(name).publish();
        }
        periodicLogs.put(new LogTopic(pub), supplier);
    }

    /**
     * Adds a new periodic log of a {@link List}.
     * <p>
     * This needs to be a completely separate method from {@link #periodicLog(String, Supplier)}
     * that takes in the type that the {@link List} holds because of some annoying stuff with
     * generics where it is impossible to get the inner type of the objects stored in a {@link
     * List} due to type erasure. We cannot correctly set up the NetworkTables publisher without
     * knowing what type it will be publishing, so we need a separate method to explicitly pass in
     * the type.
     * <p>
     * This supports logging {@link List}s of {@link Double}s, {@link Integer}s, {@link Boolean}s,
     * {@link Float}s, {@link String}s, and {@link StructSerializable} classes (if the {@link
     * Struct} is passed in, see below). Anything else passed in will be logged as a {@link List}
     * of {@link String}s.
     * <p>
     * If you are logging a {@link List} of something that is {@link StructSerializable} (for
     * example, a {@link Pose2d}, or various other WPILib classes), please use {@link
     * #periodicLogList(String, Supplier, Class, Struct)} to specify the {@link Struct} type.
     * <p>
     * If you are not logging a {@link List}, please use {@link #periodicLog(String, Supplier)} or
     * {@link #periodicLog(String, Supplier, Struct)}.
     *
     * @param name     The topic name to log under.
     * @param supplier A supplier of the values to log.
     * @param type     The class of the type that the {@link List} to be logged holds. For example,
     *                 if you wanted to log a {@link List} of {@link Double}s, you would pass in
     *                 {@code Double.class}.
     * @param <T>      The type that the supplier returns a {@link List} of.
     */
    public static <T> void periodicLogList(String name, Supplier<List<T>> supplier, Class<T> type) {
        periodicLogList(name, supplier, type, null);
    }

    /**
     * Adds a new periodic log of a {@link List} using the provided {@link Struct}.
     * <p>
     * Passing in a {@link Struct} is necessary to correctly log any {@link StructSerializable}
     * class (for example, a {@link Pose2d}, or various other WPILib classes). For most {@link
     * StructSerializable} classes, this can be obtained by getting the static {@code struct}
     * member variable from the class (for example, {@link Pose2d#struct}). If you are not logging
     * a {@link StructSerializable} class, you can leave out the {@link Struct} when calling this
     * method.
     * <p>
     * This needs to be a completely separate method from {@link #periodicLog(String, Supplier,
     * Struct)} that takes in the type that the {@link List} holds because of some annoying stuff
     * with generics where it is impossible to get the inner type of the objects stored in a {@link
     * List} due to type erasure. We cannot correctly set up the NetworkTables publisher without
     * knowing what type it will be publishing, so we need a separate method to explicitly pass in
     * the type.
     * <p>
     * This supports logging {@link List}s of {@link Double}s, {@link Integer}s, {@link Boolean}s,
     * {@link Float}s, {@link String}s, and {@link StructSerializable} classes (if the {@link
     * Struct} is passed in, see above). Anything else passed in will be logged as a {@link List}
     * of {@link String}s.
     * <p>
     * If you are not logging a {@link List}, please use {@link #periodicLog(String, Supplier)} or
     * {@link #periodicLog(String, Supplier, Struct)}.
     *
     * @param name     The topic name to log under.
     * @param supplier A supplier of the values to log.
     * @param type     The class of the type that the {@link List} to be logged holds. For example,
     *                 if you wanted to log a {@link List} of {@link Double}s, you would pass in
     *                 {@code Double.class}.
     * @param struct   The {@link Struct} for the {@link StructSerializable} class being logged.
     * @param <T>      The type that the supplier returns a {@link List} of.
     */
    public static <T> void periodicLogList(
        String name, Supplier<List<T>> supplier, Class<T> type, Struct<T> struct
    ) {
        Publisher pub;
        if (Double.class.isAssignableFrom(type)) {
            pub = netTable.getDoubleArrayTopic(name).publish();
        } else if (Integer.class.isAssignableFrom(type)) {
            pub = netTable.getIntegerArrayTopic(name).publish();
        } else if (Boolean.class.isAssignableFrom(type)) {
            pub = netTable.getBooleanArrayTopic(name).publish();
        } else if (Float.class.isAssignableFrom(type)) {
            pub = netTable.getFloatArrayTopic(name).publish();
        } else if (StructSerializable.class.isAssignableFrom(type) && struct != null) {
            pub = netTable.getStructArrayTopic(name, struct).publish();
        } else {
            pub = netTable.getStringArrayTopic(name).publish();
        }
        periodicLogs.put(new LogTopic(pub), supplier);
    }

    public static void log(Object s) {
        if (s instanceof Throwable throwable) {
            // print only the cause of the error
            while (throwable.getCause() != null) {
                throwable = throwable.getCause();
            }
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            throwable.printStackTrace(printWriter);
            var value = stringWriter.toString().replace("\r", "");
            msg.set(value);
            DataLogManager.log(value);
            Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, throwable.getClass().getName(), throwable.getMessage(), 15000, 450, -1));
        } else  if (s instanceof RobotConfig) {
            log("Pathplanner config:");
            var config = (RobotConfig) s;
            log(String.format("  Holonomic: %b",config.isHolonomic));
            log(String.format("  MOI: %f",config.MOI));
            log(String.format("  massKg: %f",config.massKG));
            log(String.format("  maxTorqueFriction: %f",config.maxTorqueFriction));
            log(String.format("  wheelFrictionForce: %f",config.wheelFrictionForce));
            log(String.format("  maxDriveVelocityMPS: %f",config.moduleConfig.maxDriveVelocityMPS));
            log(String.format("  driveCurrentLimit: %f",config.moduleConfig.driveCurrentLimit));
            log(String.format("  wheelRadiusMeters: %f",config.moduleConfig.wheelRadiusMeters));
            log(String.format("  wheelCOF: %f",config.moduleConfig.wheelCOF));
            log(String.format("  freeSpeedRadPerSec: %f",config.moduleConfig.driveMotor.freeSpeedRadPerSec));
        } else {
            var value = String.valueOf(s);
            msg.set(value);
            DataLogManager.log(value);
        }
    }

    // Will update all registered periodic loggers
    @SuppressWarnings({"unchecked", "rawtypes"})
    public static void updatePeriodic() {
        for (LogTopic entry : periodicLogs.keySet()) {
            var supplier = periodicLogs.get(entry);
            if (entry.Publisher instanceof DoublePublisher) {
                var value = (Double) supplier.get();
                ((DoublePublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof IntegerPublisher) {
                var value = (Integer) supplier.get();
                ((IntegerPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof BooleanPublisher) {
                var value = (Boolean) supplier.get();
                ((BooleanPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof FloatPublisher) {
                var value = (Float) supplier.get();
                ((FloatPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof StructPublisher) {
                var value = supplier.get();
                ((StructPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof DoubleArrayPublisher) {
                // Convert the result of the supplier from a List<Double> to a double[].
                var value = ((List<Double>) supplier.get()).stream()
                    .mapToDouble(Double::doubleValue)
                    .toArray();
                ((DoubleArrayPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof IntegerArrayPublisher) {
                // Convert the result of the supplier from a List<Integer> to an int[].
                var value = ((List<Integer>) supplier.get()).stream()
                    .mapToLong(Integer::longValue)
                    .toArray();
                ((IntegerArrayPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof BooleanArrayPublisher) {
                List<Boolean> booleanList = (List<Boolean>) supplier.get();
                // Convert the result of the supplier from a List<Boolean> to a boolean[].
                boolean[] value = new boolean[booleanList.size()];
                for (int i = 0; i < booleanList.size(); i++) {
                    // Set the boolean primitive at the corresponding index to the Boolean Object
                    // from the list. If the Boolean Object is null, default to false.
                    value[i] = Objects.requireNonNullElse(booleanList.get(i), false);
                }
                ((BooleanArrayPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof FloatArrayPublisher) {
                List<Float> floatList = (List<Float>) supplier.get();
                // Convert the result of the supplier from a List<Float> to a float[].
                float[] value = new float[floatList.size()];
                for (int i = 0; i < floatList.size(); i++) {
                    // Set the float primitive at the corresponding index to the Float Object from
                    // the list. If the Float Object is null, default to Float.NaN.
                    value[i] = Objects.requireNonNullElse(floatList.get(i), Float.NaN);
                }
                ((FloatArrayPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof StringArrayPublisher) {
                // We are using a StringArrayPublisher for a List of any type that is not covered
                // by the other array publishers (not just for Strings), so we need to get the
                // String value of the Objects in the List.
                var value = ((List<?>) supplier.get()).stream()
                    .map(String::valueOf)
                    .toArray(String[]::new);
                ((StringArrayPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof StructArrayPublisher) {
                var value = ((List) supplier.get()).toArray();
                ((StructArrayPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher == null) {
                var value = (NTSendable) supplier.get();
                SmartDashboard.putData(value);
            } else {
                var value = String.valueOf(supplier.get());
                ((StringPublisher) entry.Publisher).set(value);
            }
        }
    }
}
