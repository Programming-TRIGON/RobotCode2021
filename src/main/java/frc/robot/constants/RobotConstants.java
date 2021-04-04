package frc.robot.constants;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.components.MotorConfig;
import frc.robot.components.TBHController;
import frc.robot.constants.RobotMap.CAN.ShooterMap;
import frc.robot.subsystems.led.LedBlinkColor;
import frc.robot.subsystems.led.LedColor;
import frc.robot.utilities.*;

/**
 * All the constants to be uses for the robot
 */
public abstract class RobotConstants extends RobotMap {
    public LimelightConstants extendedLimelightConstants = new LimelightConstants();
    public LimelightConstants retractedLimelightConstants = new LimelightConstants();
    public DrivetrainConstants drivetrainConstants = new DrivetrainConstants();
    public TesterConstants testerConstants = new TesterConstants();
    public VisionConstants visionConstants = new VisionConstants();
    public ShooterConstants shooterConstants = new ShooterConstants();
    public LedConstants ledConstants = new LedConstants();
    public IntakeConstants intakeConstants = new IntakeConstants();
    public LoaderConstants loaderConstants = new LoaderConstants();
    public ClimberConstants leftClimberConstants = new ClimberConstants();
    public ClimberConstants rightClimberConstants = new ClimberConstants();
    public MotionProfilingConstants motionProfilingConstants = new MotionProfilingConstants();
    public PitcherConstants pitcherConstants = new PitcherConstants();
    public IntakeOpenerConstants intakeOpenerConstants = new IntakeOpenerConstants();
    public SpinnerConstants spinnerConstants = new SpinnerConstants();

    public class DrivetrainConstants {
        public CAN.DrivetrainMap CAN_MAP;

        public Pose2d FRONT_LEFT_LOCATION;
        public Pose2d FRONT_RIGHT_LOCATION;
        public Pose2d REAR_LEFT_LOCATION;
        public Pose2d REAR_RIGHT_LOCATION;

        public double MAX_SPEED_MPS;
        public double MAX_ROT_SPEED_RAD_S;

        public double WHEEL_DIAMETER_M;

        public SwerveConstants FRONT_LEFT_CONSTANTS;
        public SwerveConstants FRONT_RIGHT_CONSTANTS;
        public SwerveConstants REAR_LEFT_CONSTANTS;
        public SwerveConstants REAR_RIGHT_CONSTANTS;


        public SVACoefs SPEED_SVA_COEFS;
        public SVACoefs ANGLE_SVA_COEFS;
        public PIDFCoefs SPEED_PIDF_COEFS;
        public PIDFCoefs ANGLE_PIDF_COEFS;
    }

    public class LimelightConstants {
        public String DEFAULT_TABLE_KEY;
        public double DISTANCE_CALCULATION_A_COEFFICIENT;
        public double DISTANCE_CALCULATION_B_COEFFICIENT;
        public double DISTANCE_CALCULATION_C_COEFFICIENT;
        public double LIMELIGHT_ANGLE_OFFSET;
        public double LIMELIGHT_OFFSET_X;
        public double LIMELIGHT_OFFSET_Y;
        public double SHOOTER_VELOCITY_COEF_A;
        public double SHOOTER_VELOCITY_COEF_B;
        public double SHOOTER_VELOCITY_COEF_C;
    }

    public class TesterConstants {
        public int SECONDS_TO_WAIT;
        public double MOVE_POWER;
    }

    public class VisionConstants {
        public PIDFCoefs ROTATION_SETTINGS;
        public double TARGET_TIME_OUT;
    }

    public class ShooterConstants {
        public ShooterMap CAN_MAP;
        public MotorConfig RIGHT_MOTOR_CONFIG;
        public MotorConfig LEFT_MOTOR_CONFIG;
        public TBHController TBH_CONTROLLER;
        public TrigonPIDController PID_CONTROLLER;
        public SimpleMotorFeedforward SIMPLE_MOTOR_FEEDFORWARD;
        public PIDFCoefs PID_COEFS;
        public double KF_COEF_A;
        public double KF_COEF_B;
        public double SHOOTING_RAMP_RATE;
        public double TOLERANCE;
        public double TIME_AT_SETPOINT;
        // The value of the xbox at which it will cancel the ShootCMDGP allowing the
        // driver to resume driving
        public double CANCEL_CMDGP_AXIS_THRESHOLD;
        public int MAX_NUMBER_OF_BALLS;
        public int KF_CALCULATION_SAMPLE_AMOUNT;

        public double KF_TESTING_INITIAL_DESIRED_VELOCITY;
        public double KF_TESTING_VELOCITY_ACCELERATION_PER_TEST;
        public double KF_TESTING_TOLERANCE;
        public double KF_TESTING_DELTA_TOLERANCE;
        public int KF_TESTING_CALCULATION_SAMPLE_AMOUNT;
        public int KF_TESTING_TEST_AMOUNT;
    }

    public class LoaderConstants {
        public CAN.LoaderMap CAN_MAP;
        public MotorConfig MOTOR_CONFIG;
        public PIDFCoefs PID_COEFS;
        public FeedforwardConstants FEEDFORWARD_CONSTANTS;
        public double DEFAULT_SHOOTING_VELOCITY;
        public double DEFAULT_MIXING_VELOCITY;
    }

    public class LedConstants {
        public final ColorMap COLOR_MAP = new ColorMap();
        public PWM.LedMap PWM_MAP;

        public class ColorMap {
            public final LedBlinkColor SENSOR_TEST_SUCCESS = new LedBlinkColor(LedColor.Green, 5);
            public final LedBlinkColor SENSOR_TEST_FAILURE = new LedBlinkColor(LedColor.Red, 5);
            public final LedColor SHOOTER_ENABLED = LedColor.Blue;
            public final LedBlinkColor NO_TARGET = new LedBlinkColor(LedColor.Orange, 2);
            public final LedColor INTAKE_ENABLED = LedColor.Aqua;
            public final LedBlinkColor INTAKE_MOTOR_STALL = new LedBlinkColor(LedColor.Yellow, 5);
        }
    }

    public class IntakeConstants {
        public CAN.IntakeMap CAN_MAP;
        public MotorConfig MOTOR_CONFIG;
        public double DEFAULT_MOTOR_POWER;
        public double STALL_CHECK_DELAY;
        public double STALL_CURRENT_LIMIT;
    }

    public class ClimberConstants {
        public PWM.ClimberMap PWM_MAP;
        public boolean IS_INVERTED;
    }

    public class MotionProfilingConstants {
        public double MAX_VELOCITY;
        public double MAX_ACCELERATION;
        public double MAX_CENTRIPETAL_ACCELERATION;
        public double KP;
        public double REVERSE_KP;

        public TrigonPIDController X_PID_CONTROLLER;
        public TrigonPIDController Y_PID_CONTROLLER;
        public TrigonProfiledPIDController THETA_PROFILED_PID_CONTROLLER;
    }

    public class PitcherConstants {
        public PCM.PitcherMap PCM_MAP;
        public double EXTENDED_TOGGLE_ANGLE;
        public double RETRACTED_TOGGLE_ANGLE;
        public int NO_TARGET_BLINK_TIME;
    }

    public class IntakeOpenerConstants {
        public CAN.IntakeOpenerMap CAN_MAP;
        public DIO.IntakeOpenerMap DIO_MAP;
        public MotorConfig MOTOR_CONFIG;
        public double DEFAULT_OPEN_POWER;
        public double DEFAULT_CLOSE_POWER;
    }

    public class SpinnerConstants {
        public CAN.SpinnerMap CAN_MAP;
        public PCM.SpinnerMap PCM_MAP;
        public I2C.SpinnerMap I2C_MAP;
        public MotorConfig MOTOR_CONFIG;
        public double STALL_CURRENT_LIMIT;
        public double STALL_CHECK_DELAY;
        public double DEFAULT_MOTOR_POWER;
    }
}
