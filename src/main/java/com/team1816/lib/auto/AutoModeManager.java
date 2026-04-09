package com.team1816.lib.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.team1816.lib.BaseRobotState;
import com.team1816.lib.util.FieldContainer;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.*;
import java.util.function.Consumer;
import java.util.stream.Stream;

/**
 * Manages creating the auto chooser to select the auto routine. Also handles resetting the robot's
 * pose based on the auto starting pose, and drawing the path preview. Does NOT interact directly
 * with {@link CommandScheduler} or the drivetrain.
 */
public class AutoModeManager {
    private final SendableChooser<Command> autoChooser;
    /**
     * A mapping of the auto names as displayed in the {@link #autoChooser} to the path preview for
     * that auto to display on the dashboard.
     */
    private final HashMap<String, List<Pose2d>> autoPathPreviewMap = new HashMap<>();
    /**
     * A consumer that will accept a {@link Pose2d} to reset the robot's pose to.
     */
    private final Consumer<Pose2d> resetPoseConsumer;

    /**
     * Constructs the {@link AutoModeManager}, including putting the auto dropdown in
     * SmartDashboard and handling the creation of mirrored versions of autos.
     *
     * @param resetPoseConsumer A consumer that will accept a {@link Pose2d} that is the auto
     *                          starting pose to reset the robot's pose to.
     */
    public AutoModeManager(Consumer<Pose2d> resetPoseConsumer) {
        this.resetPoseConsumer = resetPoseConsumer;

        // Build the auto chooser with all the autos from PathPlanner and mirrored versions of most
        // autos.
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            autos -> autos.flatMap(auto -> {
                // Get the name of the original auto for PathPlanner to look for.
                String originalAutoName = auto.getName();
                // Don't create a mirrored version if marked with "[DM]" (Don't Mirror).
                if (originalAutoName.startsWith("[DM]")) {
                    // Remove the "[DM]" marker for the display name, and any leading spaces.
                    String modifiedAutoName = originalAutoName.substring(5).stripLeading();
                    auto.setName(modifiedAutoName);
                    addAutoPathPreviewToMap(originalAutoName, modifiedAutoName, false);
                    return Stream.of(auto);
                }
                // Rename the original auto to specify it is the left version.
                String leftAutoName = "Left " + originalAutoName;
                auto.setName(leftAutoName);
                addAutoPathPreviewToMap(originalAutoName, leftAutoName, false);
                // Create a mirrored version of the auto.
                PathPlannerAuto mirroredAuto = new PathPlannerAuto(originalAutoName, true);
                // Rename the mirrored auto to specify it is the right version.
                String rightAutoName = "Right " + originalAutoName;
                mirroredAuto.setName(rightAutoName);
                addAutoPathPreviewToMap(originalAutoName, rightAutoName, true);
                // Return both the original and the mirrored auto.
                return Stream.of(auto, mirroredAuto);
            })
        );

        // Publish the auto chooser to the SmartDashboard.
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Display the active path on the main Field2d. This is the path that PathPlanner is
        // actively running, not a path that is scheduled to run once auto starts.
        PathPlannerLogging.setLogActivePathCallback(pose2ds ->
            FieldContainer.field.getObject("activePath").setPoses(pose2ds)
        );

        // Create a trigger and use it to reset the pose to the auto start pose whenever the
        // alliance color changes.
        new Trigger(() ->
            // If the alliance is blue, defaulting to true if the alliance is empty.
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
        )
            .onChange(Commands.runOnce(() ->
                resetRobotPoseToAutoStart(autoChooser.getSelected(), false)
            ).ignoringDisable(true));

        // Register a listener to reset the pose to the auto start pose whenever the selected auto
        // changes.
        autoChooser.onChange(autoCommand -> resetRobotPoseToAutoStart(autoCommand, false));

        // Use the autonomous trigger to force reset the pose to the auto start pose whenever auto
        // starts, to make sure the pose is set correctly at the beginning of auto.
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() ->
            resetRobotPoseToAutoStart(autoChooser.getSelected(), true)
        ).ignoringDisable(true));
    }

    /**
     * Resets the robot pose using the {@link #resetPoseConsumer} to the start pose of the passed
     * in auto command.
     *
     * @param autoCommand The auto command to use the start pose of.
     * @param forceResetPose If the pose should be reset regardless of whether the match has
     *                       already started.
     */
    private void resetRobotPoseToAutoStart(Command autoCommand, boolean forceResetPose) {
        // Only reset the pose if auto has not started, or if forceResetPose is true. This is to
        // prevent resetting the pose in the middle of a match. This is based on if auto has ever
        // started, rather than just checking if we are currently in teleop or auto, because we
        // want to prevent the pose resetting even during the disabled period between auto and
        // teleop.
        if (!BaseRobotState.hasAutoStarted || forceResetPose) {
            // Get the starting pose for the auto. If this is the default None auto, it will just
            // be an empty Command rather than a PathPlannerAuto, so use Pose2d.kZero in that case.
            Pose2d startingPose = autoCommand instanceof PathPlannerAuto
                ? ((PathPlannerAuto) autoCommand).getStartingPose()
                : Pose2d.kZero;
            // Flip the starting pose based on alliance, defaulting to blue if the alliance is empty.
            startingPose = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? startingPose
                : FlippingUtil.flipFieldPose(startingPose);
            // Call the consumer to reset the robot's pose.
            resetPoseConsumer.accept(startingPose);
        }
    }

    /**
     * Updates the display of the selected auto path on the field.
     */
    public void updateAutoPathPreviewDisplay() {
        String autoName = autoChooser.getSelected().getName();
        List<Pose2d> path = autoPathPreviewMap.get(autoName);
        if (path != null) {
            // Flip the path based on alliance, defaulting to blue if the alliance is empty.
            path = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? path
                : path.stream().map(FlippingUtil::flipFieldPose).toList();
            // The number of Pose2ds to move through in the path per second.
            double pose2dsPerSecond = 300;
            // Zero-indexed progress through the path to display. Use the timer to have it count
            // up, using modulus to loop back to the start when the path ends.
            int pathProgress = (int) ((Timer.getFPGATimestamp() * pose2dsPerSecond) % (path.size()));
            // Show the path on the field up to the pathProgress.
            FieldContainer.field.getObject("autoPathPreview").setPoses(
                path.stream().limit(pathProgress + 1).toList()
            );
        }
        else {
            // Clear the poses if no valid path was found.
            clearAutoPathPreviewDisplay();
        }
    }

    /**
     * Clear the display of the selected auto path on the field.
     */
    public void clearAutoPathPreviewDisplay() {
        FieldContainer.field.getObject("autoPathPreview").setPoses();
    }

    /**
     * Adds the paths from the auto to the {@link #autoPathPreviewMap} so we can look it up later for
     * displaying on the field using the name stored in the {@link #autoChooser}. This is necessary
     * because we modify the auto names when we build the auto chooser, and the only way to get the
     * paths is to load the auto from the file, which requires having the original auto file name.
     *
     * @param originalAutoName The name of the auto file that contains the paths.
     * @param modifiedAutoName The modified name of the auto that will be used later to find the
     *                         paths from the map.
     * @param mirror If the paths stored should be mirrored to the other side of the current
     *               alliance from the original paths stored in the auto file.
     */
    private void addAutoPathPreviewToMap(String originalAutoName, String modifiedAutoName, boolean mirror) {
        try {
            // Load the paths for the auto.
            List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(originalAutoName);

            // Combine the paths into a single list of Pose2ds, with repeated Pose2ds at the end of
            // each path and at the very end to create pauses when running through the path with
            // the preview.
            // The number of times to repeat the last Pose2d in each path.
            int pathEndRepeats = 5;
            // The number of times to repeat the last Pose2d in the whole auto.
            int autoEndRepeats = 10;
            List<Pose2d> combinedPath = new ArrayList<>();
            for (int i = 0; i < pathPlannerPaths.size(); i++) {
                PathPlannerPath path = pathPlannerPaths.get(i);

                // Mirror the path if necessary.
                if (mirror) path = path.mirrorPath();

                // Get the list of poses representing the path.
                List<Pose2d> pathPoses = path.getPathPoses();
                if (pathPoses.isEmpty()) continue;

                // Add all the poses from the individual path to the combined path.
                combinedPath.addAll(pathPoses);

                // Repeat the last pose in the individual path to create a pause at the end of each
                // path.
                Pose2d lastPose = pathPoses.get(pathPoses.size() - 1);
                combinedPath.addAll(Collections.nCopies(pathEndRepeats, lastPose));

                // If this is the last path in the auto, repeat the last pose in the path to create
                // a longer pause at the very end of the auto.
                if (i == pathPlannerPaths.size() - 1) {
                    combinedPath.addAll(Collections.nCopies(autoEndRepeats, lastPose));
                }
            }

            autoPathPreviewMap.put(
                modifiedAutoName,
                combinedPath
            );
        } catch (IOException e) {
            GreenLogger.log(e);
            GreenLogger.log("IOException while getting path group. See above stack trace for details.");
        } catch (ParseException e) {
            GreenLogger.log(e);
            GreenLogger.log("ParseException while getting path group. See above stack trace for details.");
        }
    }

    /**
     * Retrieves currently selected auto from the {@link #autoChooser}.
     */
    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}
