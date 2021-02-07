package frc.robot.constants;

import frc.robot.components.MotorConfig;
import frc.robot.constants.RobotMap.CAN.ShooterMap;
import frc.robot.subsystems.led.LedBlinkColor;
import frc.robot.subsystems.led.LedColor;
import frc.robot.utilities.PIDCoefs;

import java.util.function.DoubleSupplier;

/**
 * All the constants to be uses for the robot
 */
public abstract class RobotConstants extends RobotMap {
    public LimelightConstants limelightConstants = new LimelightConstants();
    public TesterConstants testerConstants = new TesterConstants();
    public VisionConstants visionConstants = new VisionConstants();
    public ShooterConstants shooterConstants = new ShooterConstants();
    public LedConstants ledConstants = new LedConstants();
    public IntakeConstants intakeConstants = new IntakeConstants();
    public TriggerConstants triggerConstants = new TriggerConstants();
    public ClimberConstants leftClimberConstants = new ClimberConstants();
    public ClimberConstants rightClimberConstants = new ClimberConstants();

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
    }

    public class VisionConstants {
        public PIDCoefs ROTATION_SETTINGS;
        public double TARGET_TIME_OUT;
    }

    public class ShooterConstants {
        public ShooterMap CAN_MAP;
        public MotorConfig RIGHT_MOTOR_CONFIG;
        public MotorConfig LEFT_MOTOR_CONFIG;
    }

    public class TriggerConstants {
        public CAN.TriggerMap CAN_MAP;
        public MotorConfig MOTOR_CONFIG;
        public PIDCoefs PID_COEFS;
    }

    public class LedConstants {
        public PWM.LedMap PWM_MAP;
        public final ColorMap COLOR_MAP = new ColorMap();

        public class ColorMap {
            public final LedBlinkColor SENSOR_TEST_SUCCESS = new LedBlinkColor(LedColor.Green, 5);
            public final LedBlinkColor SENSOR_TEST_FAILURE = new LedBlinkColor(LedColor.Red, 5);
        }
    }

    public class IntakeConstants {
        public CAN.IntakeMap CAN_MAP;
        public MotorConfig MOTOR_CONFIG;
        public double DEFAULT_MOTOR_POWER;
        public double STALL_CHECK_DELAY;
        public double STALL_CURRENT_LIMIT;
        public int STALL_BLINK_AMOUNT;
    }

    public class ClimberConstants {
        public PWM.ClimberMap PWM_MAP;
        public boolean IS_INVERTED;
    }
}
