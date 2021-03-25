package frc.robot.motion_profiling;

/**
 * This enum represent and hold the instances of auto paths
 */
public enum AutoPath {
    FacingPowerPortToTrenchStart, InLineWithTrenchToTrenchStart, InTrench,
    ReverseInTrench, RightOfPortToMiddleField, FacingPowerPortToMiddleField,
    InitLineToEnemyTrench, EnemyTrenchToPort,
    SimpleAutoToTrench, TurnFromTrenchToPort;

    private final Path path;

    AutoPath(Path path) {
        this.path = path;
    }

    AutoPath() {
        path = new Path(name() + ".wpilib.json");
    }

    AutoPath(String name) {
        path = new Path(name + ".wpilib.json");
    }

    public Path getPath() {
        return path;
    }
}