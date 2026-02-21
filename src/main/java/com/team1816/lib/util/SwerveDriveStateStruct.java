package com.team1816.lib.util;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;

import java.nio.ByteBuffer;

/**
 * A {@link Struct} implementation for CTRE's {@link
 * com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState} to allow for easier sending across
 * the NetworkTables for logging.
 */
public class SwerveDriveStateStruct implements Struct<SwerveDrivetrain.SwerveDriveState> {
    private final int numModules;

    /**
     * Constructs the {@link Struct} implementation.
     *
     * @param numModules The number of modules of the {@link
     *                   com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState} objects this
     *                   serializer processes.
     */
    public SwerveDriveStateStruct(int numModules) {
        this.numModules = numModules;
    }

    @Override
    public Class<SwerveDrivetrain.SwerveDriveState> getTypeClass() {
        return  SwerveDrivetrain.SwerveDriveState.class;
    }

    @Override
    public String getTypeName() {
        return "SwerveDriveState__" + numModules;
    }

    @Override
    public int getSize() {
        return Pose2d.struct.getSize() +
            ChassisSpeeds.struct.getSize() +
            (SwerveModuleState.struct.getSize() * numModules) +
            (SwerveModuleState.struct.getSize() * numModules) +
            (SwerveModulePosition.struct.getSize() * numModules) +
            Rotation2d.struct.getSize() +
            kSizeDouble +
            kSizeDouble +
            kSizeInt32 +
            kSizeInt32;
    }

    @Override
    public String getSchema() {
        return "Pose2d Pose;" +
            "ChassisSpeeds Speeds;" +
            "SwerveModuleState ModuleStates[" + numModules + "];" +
            "SwerveModuleState ModuleTargets[" + numModules + "];" +
            "SwerveModulePosition ModulePositions["  + numModules + "];" +
            "Rotation2d RawHeading;" +
            "double Timestamp;" +
            "double OdometryPeriod;" +
            "int32 SuccessfulDaqs;" +
            "int32 FailedDaqs";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] {
            Pose2d.struct,
            ChassisSpeeds.struct,
            SwerveModuleState.struct,
            SwerveModulePosition.struct,
            Rotation2d.struct
        };
    }

    @Override
    public SwerveDrivetrain.SwerveDriveState unpack(ByteBuffer bb) {
        final var toReturn = new SwerveDrivetrain.SwerveDriveState();
        toReturn.Pose = Pose2d.struct.unpack(bb);
        toReturn.Speeds = ChassisSpeeds.struct.unpack(bb);
        toReturn.ModuleStates = Struct.unpackArray(bb, numModules, SwerveModuleState.struct);
        toReturn.ModuleTargets = Struct.unpackArray(bb, numModules, SwerveModuleState.struct);
        toReturn.ModulePositions = Struct.unpackArray(bb, numModules, SwerveModulePosition.struct);
        toReturn.RawHeading = Rotation2d.struct.unpack(bb);
        toReturn.Timestamp = bb.getDouble();
        toReturn.OdometryPeriod = bb.getDouble();
        toReturn.SuccessfulDaqs = bb.getInt();
        toReturn.FailedDaqs = bb.getInt();
        return toReturn;
    }

    @Override
    public void pack(ByteBuffer bb, SwerveDrivetrain.SwerveDriveState value) {
        Pose2d.struct.pack(bb, value.Pose);
        ChassisSpeeds.struct.pack(bb, value.Speeds);
        Struct.packArray(bb, value.ModuleStates, SwerveModuleState.struct);
        Struct.packArray(bb, value.ModuleTargets, SwerveModuleState.struct);
        Struct.packArray(bb, value.ModulePositions, SwerveModulePosition.struct);
        Rotation2d.struct.pack(bb, value.RawHeading);
        bb.putDouble(value.Timestamp);
        bb.putDouble(value.OdometryPeriod);
        bb.putInt(value.SuccessfulDaqs);
        bb.putInt(value.FailedDaqs);
    }
}
