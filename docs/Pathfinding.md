# Pathfinding

A simple guide on working with pathfinding on the robot.

Paths are configured completely through the YAML; no code changes are needed.

## Defining constraints

Define default constraints for _all_ the paths. You are able to override these constraints for specific paths at your discretion.

Under the `autopathing` section of the robot's YAML, there should be a `constraints` section.

```yaml
autopathing:
    constraints:
        maxVelocity: [REQUIRED; maximum velocity in m/s]
        maxAccel: [REQUIRED; maximum acceleration in m/s²]
        maxAngularVelocity: [REQUIRED; maximum angular velocity in degrees/s]
        maxAngularAccel: [REQUIRED; maximum angular acceleration in degrees/s²]
```

## Adding/Modifying Paths

Under the `autopathing` section of the robot's YAML, you will see the `paths` section. To add a path, simply add a new section, with the name of the path and several requisite items for the target robot state.

```yaml
path_name:
    x: [REQUIRED; target x position in meters]
    y: [REQUIRED; target y position in meters]
    rotation: [REQUIRED; target rotation in degrees]

    flippable: [NOT REQUIRED; boolean of whether the target position should change based on alliance colour. default true]
    targetVelocity: [NOT REQUIRED; the target end velocity. default 0]

    constraints: # NOT REQUIRED; overrides default constraints. follows exact same pattern as defaults. all fields are required.
```

## Running a Path

Simply enter SmartDashboard and use the `Pathfind Target` dropdown. The dropdown will automatically populate with all the configured path targets. Press the designated key (found in `RobotContainer#configureBindings`) to begin pathing.

Pathing will stop when it encounters any joystick input.

## Known issues

- Targets that are very close to the current robot position can cause a silent, uncatchable exception to be thrown. We are still looking into this.
