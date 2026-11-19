package com.team1816.lib.util;

import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.util.struct.Struct;

public class LogTopic {
    public final String name;
    public final Publisher Publisher;
    public final Struct<?> struct;
    /** False for CTRE device signals that SignalLogger already records. */
    public final boolean writeToSignalLog;

    public LogTopic(String name, Publisher publisher, Struct<?> struct, boolean writeToSignalLog) {
        this.name = name;
        this.Publisher = publisher;
        this.struct = struct;
        this.writeToSignalLog = writeToSignalLog;
    }
}
