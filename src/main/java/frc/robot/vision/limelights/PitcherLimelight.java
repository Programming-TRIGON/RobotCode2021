package frc.robot.vision.limelights;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants.LimelightConstants;
import frc.robot.subsystems.pitcher.PitcherSS;

/**
 * Class for Limelight when it's on the pitcher.
 * It gets two sets of constants, one for when the hood is extended and one for when it is retracted.
 */
public class PitcherLimelight extends VanillaLimelight {
    private final PitcherSS pitcherSS;
    private final LimelightConstants extendedConstants = constants;
    private final LimelightConstants retractedConstants;

    /**
     * @param tableKey           the key of the limelight - if it was changed.
     * @param extendedConstants  constants for when the hood is extended
     * @param retractedConstants constants for when the hood is retracted
     */
    public PitcherLimelight(String tableKey, LimelightConstants extendedConstants, LimelightConstants retractedConstants, PitcherSS pitcherSS) {
        super(tableKey, extendedConstants);
        this.pitcherSS = pitcherSS;
        this.retractedConstants = retractedConstants;
    }

    /**
     * @param extendedConstants  constants for when the hood is extended
     * @param retractedConstants constants for when the hood is retracted
     */
    public PitcherLimelight(LimelightConstants extendedConstants, LimelightConstants retractedConstants, PitcherSS pitcherSS) {
        this(extendedConstants.DEFAULT_TABLE_KEY, extendedConstants, retractedConstants, pitcherSS);
    }

    /**
     * @return is the hood extended (true=extended false=retracted)
     */
    public boolean hoodExtended() {
        return pitcherSS.getSolenoidState();
    }

    /**
     * @param position to be set to the hood (true=extended false=retracted)
     */
    public void setHoodState(boolean position) {
        pitcherSS.setSolenoidState(position);
    }

    /**
     * @return The distance between the target and the limelight
     */
    // TODO: set real function
    public double getTargetDistance() {
        double y = getTy();
        if (hoodExtended())
            return extendedConstants.DISTANCE_CALCULATION_A_COEFFICIENT * Math.pow(y, 2)
                    + extendedConstants.DISTANCE_CALCULATION_B_COEFFICIENT * y
                    + extendedConstants.DISTANCE_CALCULATION_C_COEFFICIENT;
        else
            return retractedConstants.DISTANCE_CALCULATION_A_COEFFICIENT * Math.pow(y, 2)
                    + retractedConstants.DISTANCE_CALCULATION_B_COEFFICIENT * y
                    + retractedConstants.DISTANCE_CALCULATION_C_COEFFICIENT;
    }

    /**
     * Calculates the desired desiredVelocity to set the motors based on the height
     * at which the limelight sees the target
     *
     * @return the desired desiredVelocity of the motors
     */
    // TODO: Set correct calculation based on function chosen for calculation.
    public double calculateDesiredShooterVelocity() {
        double y = getTy();
        if (hoodExtended())
            return extendedConstants.SHOOTER_VELOCITY_COEF_A * Math.pow(y, 2) + extendedConstants.SHOOTER_VELOCITY_COEF_B * y
                    + extendedConstants.SHOOTER_VELOCITY_COEF_C;
        else
            return retractedConstants.SHOOTER_VELOCITY_COEF_A * Math.pow(y, 2) + retractedConstants.SHOOTER_VELOCITY_COEF_B * y
                    + retractedConstants.SHOOTER_VELOCITY_COEF_C;
    }

    /**
     * @return the vector between the middle of the robot and the target.
     */
    private Vector2d calculateVector() {
        // This is the vector from the limelight to the target.
        Vector2d limelightToTarget = new Vector2d(getTargetDistance(), 0);
        if (hoodExtended()) {
            limelightToTarget.rotate(getTx() + extendedConstants.LIMELIGHT_ANGLE_OFFSET);
            // The offset is subtracted from the limelightToTarget vector in order to get
            // the final vector.
            return new Vector2d(limelightToTarget.x - extendedConstants.LIMELIGHT_OFFSET_X,
                    limelightToTarget.y - extendedConstants.LIMELIGHT_OFFSET_Y);
        }
        else {
            limelightToTarget.rotate(getTx() + retractedConstants.LIMELIGHT_ANGLE_OFFSET);
            // The offset is subtracted from the limelightToTarget vector in order to get
            // the final vector.
            return new Vector2d(limelightToTarget.x - retractedConstants.LIMELIGHT_OFFSET_X,
                    limelightToTarget.y - retractedConstants.LIMELIGHT_OFFSET_Y);
        }
    }
}
