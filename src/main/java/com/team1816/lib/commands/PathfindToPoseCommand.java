package com.team1816.lib.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.Optional;

public class PathfindToPoseCommand extends GreenCommand {
    public PathfindToPoseCommand(String pathName,
                                 Pose2d targetPose,
                                 PathConstraints constraints,
                                 boolean flippable,
                                 double targetVelocity) {
        this.pathName = pathName;
        this.targetPose = targetPose;
        this.flippable = flippable;
        this.targetVelocity = targetVelocity;
        this.constraints = constraints;
    }

    private Command internalCommand;

    private final Pose2d targetPose;
    private final String pathName;
    private final boolean flippable;
    private final double targetVelocity;
    private final PathConstraints constraints;

    // Used to detect inputs from controller to cancel pathing
    private final CommandXboxController controller = new CommandXboxController(0);

    @Override
    public void initialize() {
        if (shouldFlip()) {
            GreenLogger.log("Starting path " + pathName + " (flipped)");
        } else {
            GreenLogger.log("Starting path " + pathName);
        }

        internalCommand = AutoBuilder.pathfindToPose(
            getTargetPose(),
            constraints,
            targetVelocity
        );
        internalCommand.initialize();
    }

    @Override
    public void execute() {
        if (controllerInputDetected()) {
            return;
        }

        internalCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (shouldFlip()) {
            GreenLogger.log("Finished path " + pathName + " (flipped)");
        } else {
            GreenLogger.log("Finished path " + pathName);
        }
        internalCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return controllerInputDetected() || internalCommand.isFinished();
    }

    public Pose2d getTargetPose() {
        return shouldFlip() ? FlippingUtil.flipFieldPose(targetPose) : targetPose;
    }

    public String getPathName() {
        return pathName;
    }

    private boolean shouldFlip() {
        if (!flippable) return false;

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
    }

    private boolean controllerInputDetected() {
        return controller.getLeftY()  != 0
            || controller.getLeftX()  != 0
            || controller.getRightX() != 0;
    }
}
