package com.team1816.lib.auto;

import com.team1816.lib.Singleton;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.util.GreenLogger;
import com.team1816.lib.commands.PathfindToPoseCommand;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PathfindManager {
    public PathfindManager() {
        commandChooser = new SendableChooser<>();
        SmartDashboard.putData("Pathfind Target", commandChooser);

        commandChooser.setDefaultOption("None", null);

        NetworkTable netTable = NetworkTableInstance.getDefault().getTable("");

        commandChooser.onChange(c -> {
            if (c == null) {
                if (targetPosePub != null) {
                    targetPosePub.close();
                    targetPosePub = null;
                }
            } else {
                if (targetPosePub == null) {
                    targetPosePub = netTable.getDoubleArrayTopic("Field/PathfindTarget").publish();
                }
                targetPosePub.set(new double[]{
                    c.getTargetPose().getX(),
                    c.getTargetPose().getY(),
                    c.getTargetPose().getRotation().getDegrees()
                });
            }
        });

        RobotFactory factory = Singleton.get(RobotFactory.class);
        assert factory != null;
        // Iterate through paths and add them to the path manager
        factory.getPaths().forEach(this::addPath);
    }

    private final SendableChooser<PathfindToPoseCommand> commandChooser;
    private PathfindToPoseCommand currentPathCommand = null;

    private static DoubleArrayPublisher targetPosePub;

    public void addPath(PathfindToPoseCommand cmd) {
        commandChooser.addOption(cmd.getPathName(), cmd);
    }

    public void startPathfinding() {
        currentPathCommand =  commandChooser.getSelected();

        if (currentPathCommand == null) {
            GreenLogger.log("Attempted to start pathing with no path selected");
        } else {
            CommandScheduler.getInstance().schedule(currentPathCommand);
        }
    }

    public void stopPathfinding() {
        if (currentPathCommand != null) {
            currentPathCommand.cancel();
            currentPathCommand = null;
        }
    }
}
