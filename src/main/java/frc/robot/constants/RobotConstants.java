package frc.robot.constants;

import frc.robot.utilities.PIDCoefs;

/**
 * All the constants to be uses for the robot
 */
public abstract class RobotConstants extends RobotMap {
    public LimelightConstants limelightConstants = new LimelightConstants();

    public TesterConstants testerConstants = new TesterConstants();
    public VisionConstants visionConstants = new VisionConstants();


    public class LimelightConstants {
        public double DISTANCE_CALCULATION_A_COEFFICIENT;
        public double DISTANCE_CALCULATION_B_COEFFICIENT;
        public double DISTANCE_CALCULATION_C_COEFFICIENT;
        public double LIMELIGHT_ANGLE_OFFSET;
        public double LIMELIGHT_OFFSET_X;
        public double LIMELIGHT_OFFSET_Y;
        public String DEFAULT_TABLE_KEY;
    }

    public class TesterConstants {
        public int SECONDS_TO_WAIT;
        public double MOVE_POWER;
        public int LED_BLINK_AMOUNT;
    }

    public class VisionConstants {
        public PIDCoefs ROTATION_SETTINGS;
        public double TARGET_TIME_OUT;

    }

}
