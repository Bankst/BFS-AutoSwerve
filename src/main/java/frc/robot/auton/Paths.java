package frc.robot.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import static frc.robot.Constants.AutoConstants.*;

public class Paths {
    public static final class PP {
        public static final PathConstraints kPathConstraints = new PathConstraints(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared);
        public static final PathPlannerTrajectory testPath = PathPlanner.loadPath(
            "test", kPathConstraints);
    }
}
