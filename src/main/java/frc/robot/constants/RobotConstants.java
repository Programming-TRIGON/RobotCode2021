package frc.robot.constants;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.components.MotorConfig;
import frc.robot.constants.RobotMap.CAN.ShooterMap;
import frc.robot.utilities.PIDCoefs;

/**
 * All the constants to be uses for the robot
 */
public abstract class RobotConstants extends RobotMap {

	public LimelightConstants limelightConstants = new LimelightConstants();
	public DrivetrainConstants drivetrainConstants = new DrivetrainConstants();
	public TesterConstants testerConstants = new TesterConstants();
	public VisionConstants visionConstants = new VisionConstants();
	public ShooterConstants shooterConstants = new ShooterConstants();
	public LedConstants ledConstants = new LedConstants();

	public static class swerveConstants {
		public static final FeedbackDevice
				SWERVE_MODULE_ANGLE_MOTOR_FEEDBACK_DEVICE = FeedbackDevice.CTRE_MagEncoder_Absolute;
		public static final int
				TALON_FX_TICKS_PER_REVOLUTION = 4096;
		public static final MotorConfig
				SWERVE_MODULE_SPEED_MOTOR_CONFIG = new MotorConfig(),
				SWERVE_MODULE_ANGLE_MOTOR_CONFIG = new MotorConfig();
		public static final PIDCoefs
				SWERVE_MODULE_SPEED_PID_COEFS = new PIDCoefs(1, 1, 1, 1, 1),
				SWERVE_MODULE_ANGLE_PID_COEFS = new PIDCoefs(1, 1, 1, 1, 1);


	}

	public class DrivetrainConstants {
		public CAN.DrivetrainMap canDrivetrainMap;
		public Pose2d
				FRONT_LEFT_LOCATION,
				FRONT_RIGHT_LOCATION,
				REAR_LEFT_LOCATION,
				REAR_RIGHT_LOCATION;
		public double
				MAX_SPEED,
				MAX_ROT_SPEED;
		public MotorConfig
				ANGLE_MOTOR_CONFIG,
				SPEED_MOTOR_CONFIG;

		public double WHEEL_RADIUS;
	}

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


	public class ShooterConstants {
		public ShooterMap canShooterMap;
		public MotorConfig RIGHT_MOTOR_CONFIG;
		public MotorConfig LEFT_MOTOR_CONFIG;
		public int CENTISECONDS_IN_MINUTE;
		public int TICKS_PER_REVOLUTION;
	}

	public class LedConstants {
		public PWM.LedMap LED_PWM_MAP;
	}
}
