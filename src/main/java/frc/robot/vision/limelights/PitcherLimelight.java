package frc.robot.vision.limelights;

import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.constants.RobotConstants.LimelightConstants;
import frc.robot.subsystems.pitcher.PitcherSS;

public class PitcherLimelight extends Limelight {
    private LimelightConstants constants;
    private PitcherSS pitcherSS;

    public PitcherLimelight(String tableKey, LimelightConstants constants, PitcherSS pitcherSS) {
        super(tableKey, constants);
        this.constants = constants;
        this.pitcherSS = pitcherSS;
    }

    public PitcherLimelight(LimelightConstants constants, PitcherSS pitcherSS) {
        this(constants.DEFAULT_TABLE_KEY, constants, pitcherSS);
    }

    /**
     * @return The distance between the target and the limelight
     */
    // TODO: set real function
    public double getDistanceFromLimelight() {
        double y = getTy();
        return constants.DISTANCE_CALCULATION_A_COEFFICIENT * Math.pow(y, 2)
                + constants.DISTANCE_CALCULATION_B_COEFFICIENT * y
                + constants.DISTANCE_CALCULATION_C_COEFFICIENT;
    }

    /**
     * @return The distance between the target and the middle of the robot
     */
    public double getDistance() {
        return calculateVector().magnitude();
    }

    /**
     * @return the angle from the middle of the robot to the target
     */
    public double getAngle() {
        Vector2d vector = calculateVector();
        return Math.toDegrees(Math.atan(vector.y / vector.x));
    }

    /**
     * @return is the hood is extended (true=extended false=retracted)
     */
    public boolean isHoodExtended() {
        return pitcherSS.getSolenoidState();
    }

    /**
     * @param position to be set to the hood (true=extended false=retracted)
     */
    public void setHood(boolean position) {
        pitcherSS.setSolenoidState(position);
    }

    /**
     * @return the vector between the middle of the robot and the target.
     */
    private Vector2d calculateVector() {
        // This is the vector from the limelight to the target.
        Vector2d limelightToTarget = new Vector2d(getDistanceFromLimelight(), 0);
        limelightToTarget.rotate(getTx() + constants.LIMELIGHT_ANGLE_OFFSET);
        // The offset is subtracted from the limelightToTarget vector in order to get
        // the final vector.
        return new Vector2d(limelightToTarget.x - constants.LIMELIGHT_OFFSET_X,
                limelightToTarget.y - constants.LIMELIGHT_OFFSET_Y);
    }
}
