package frc.robot.motion_profiling;

import frc.robot.constants.RobotConstants.MotionProfilingConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSS;

/**
 * This enum represent and hold the instances of auto paths
 */
public enum AutoPath {
    FacingPowerPortToTrenchStart, InLineWithTrenchToTrenchStart, InTrench, ReverseInTrench, RightOfPortToMiddleField,
    FacingPowerPortToMiddleField, InitLineToEnemyTrench, EnemyTrenchToPort, SimpleAutoToTrench, TurnFromTrenchToPort,
    Test(new Waypoint(0,0,0), new Waypoint(1.5,1,90),new Waypoint(3,0,270));

    private final Path path;
    private final Waypoint[] waypoints;

    AutoPath(Waypoint... waypoints) {
        this.path = null;
        this.waypoints = waypoints;
    }
    AutoPath() {
        path = new Path(name() + ".wpilib.json");
        this.waypoints = null;
    }

    AutoPath(String name) {
        path = new Path(name + ".wpilib.json");
        this.waypoints = null;
    }

    public Path getPath(DrivetrainSS drivetrainSS, MotionProfilingConstants constants) {
        if (path == null)
            return new Path(drivetrainSS, constants, waypoints);
        else
            return path;
    }
}