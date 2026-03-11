package com.team1816.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * This class holds the main {@link Field2d} object for displaying the locations of objects on the
 * field in simulation and in Elastic.
 */
public class FieldContainer {
    /**
     * The main {@link Field2d} object for displaying the locations of objects on the field in
     * simulation and in Elastic. This {@link Field2d} will be sent across the Network Tables at
     * SmartDashboard/Field.
     */
    public static final Field2d field = new Field2d();

    static {
        // Send the Field2d across the Network Tables to make it visible in the sim and Elastic.
        GreenLogger.periodicLog("Field", () -> field);
    }
}
