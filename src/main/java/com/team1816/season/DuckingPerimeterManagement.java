package com.team1816.season;

import com.team1816.lib.BaseRobotState;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DuckingPerimeterManagement {
    private final Translation2d upperLeftCorner = new Translation2d(0, 5); //Need to set
    private final Translation2d lowerRightCorner = new Translation2d(5,0);
    private final Rectangle2d perimeter = new Rectangle2d(upperLeftCorner, lowerRightCorner);
    private boolean isInsidePerimeter;

    public boolean isInsidePerimeter() {
        return isInsidePerimeter;
    }

    public boolean isNotInsidePerimeter() {return !isInsidePerimeter;}

    public boolean checkIfInsidePerimeter() {
        if(perimeter.contains(new Translation2d(BaseRobotState.swerveDriveState.Pose.getX(), BaseRobotState.swerveDriveState.Pose.getY()))) {
            return isInsidePerimeter = true;
        }
        else {
            return isInsidePerimeter = false;
        }
    }
}
