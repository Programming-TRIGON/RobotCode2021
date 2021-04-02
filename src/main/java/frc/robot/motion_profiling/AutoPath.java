package frc.robot.motion_profiling;

import frc.robot.constants.RobotConstants.MotionProfilingConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSS;

/**
 * This enum represent and hold the instances of auto paths
 */
public enum AutoPath {
    FacingPowerPortToTrenchStart, InLineWithTrenchToTrenchStart, InTrench, ReverseInTrench, RightOfPortToMiddleField,
    FacingPowerPortToMiddleField, InitLineToEnemyTrench, EnemyTrenchToPort, SimpleAutoToTrench, TurnFromTrenchToPort,
    Test(new Waypoint(0, 0, 0), new Waypoint(1, 0, 90)),
    AutoNavBarrel(
            new Waypoint(2.145145583, 1.230035605, 0),
            new Waypoint(1.759729355, 4.526358605, 0),
            new Waypoint(1.181605014, 3.157116743, 0),
            new Waypoint(2.013292663, 3.654100826, 0),
            new Waypoint(3.139113749, 7.041706616, 0),
            new Waypoint(3.382534524, 5.378331318, 0),
            new Waypoint(1.384455660, 6.483867340, 0),
            new Waypoint(0.806331318, 7.751683878, 0),
            new Waypoint(2.084290389, 8.390663413, 0),
            new Waypoint(2.743554989, 1.483598913, 0));

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
