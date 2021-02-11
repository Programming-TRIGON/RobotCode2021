package frc.robot.vision;

/**
 * This enum represent a potential targets that the robot
 * can follow using vision.
 * Each of the targets has an index,
 * representing what pipeline limelight should use for finding it.
 */
public enum Target {
    PowerPort(0), Feeder(1), PowerCell(2);

    private final int index;

    Target(int index) {
        this.index = index;
    }

    public int getIndex() {
        return index;
    }

}
