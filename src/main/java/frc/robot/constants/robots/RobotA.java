package frc.robot.constants.robots;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.components.*;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.FeedforwardConstants;
import frc.robot.utilities.PIDFCoefs;
import frc.robot.utilities.SVACoefs;
import frc.robot.utilities.SwerveConstants;
import frc.robot.utilities.SwerveConstants.StaticSwerveConstants;

/**
 * instantiates the robot constants
 */
public class RobotA extends RobotConstants {

    // TODO: Set Constants
    public RobotA() {
        /* Robot constants */

        // Drivetrain constants
        drivetrainConstants.CAN_MAP = can.drivetrainMap;
        drivetrainConstants.FRONT_LEFT_LOCATION = new Pose2d(0.29765, -0.29765, Rotation2d.fromDegrees(168));
        drivetrainConstants.FRONT_RIGHT_LOCATION = new Pose2d(0.29765, 0.29765, Rotation2d.fromDegrees(288));
        drivetrainConstants.REAR_LEFT_LOCATION = new Pose2d(-0.29765, -0.29765, Rotation2d.fromDegrees(72.7));
        drivetrainConstants.REAR_RIGHT_LOCATION = new Pose2d(-0.29765, 0.29765, Rotation2d.fromDegrees(335));
        drivetrainConstants.WHEEL_DIAMETER_M = 0.05; // in meters
        drivetrainConstants.MAX_SPEED_MPS = 5; // in m/s
        drivetrainConstants.MAX_ROT_SPEED_RAD_S = 10; // in rad/s

        StaticSwerveConstants.ANGLE_TICKS_PER_REVOLUTION = 4096;
        StaticSwerveConstants.SPEED_MOTOR_TICKS_PER_REVOLUTION = 2048;
        StaticSwerveConstants.ANGLE_DEFAULT_CONFIG = new MotorConfig(.0, NeutralMode.Brake, 0);
        StaticSwerveConstants.SPEED_DEFAULT_CONFIG = new MotorConfig(.80, NeutralMode.Brake, 0);
        StaticSwerveConstants.SPEED_GEAR_RATION = 6.86;

        // Limelight Constants
        limelightConstants.DISTANCE_CALCULATION_A_COEFFICIENT = 1;
        limelightConstants.DISTANCE_CALCULATION_B_COEFFICIENT = 1;
        limelightConstants.DISTANCE_CALCULATION_C_COEFFICIENT = 1;
        limelightConstants.LIMELIGHT_ANGLE_OFFSET = 1;
        limelightConstants.LIMELIGHT_OFFSET_X = 1;
        limelightConstants.LIMELIGHT_OFFSET_Y = 1;
        limelightConstants.DEFAULT_TABLE_KEY = "limelight";

        // Sensor check constants
        testerConstants.MOVE_POWER = 1;
        testerConstants.SECONDS_TO_WAIT = 3;

        // Vision Constants
        visionConstants.ROTATION_SETTINGS = new PIDFCoefs(0, 0, 0, 0, 0);
        visionConstants.TARGET_TIME_OUT = 0.1;

        // Trigger Constants
        triggerConstants.CAN_MAP = can.triggerMap;
        triggerConstants.MOTOR_CONFIG = new MotorConfig();
        triggerConstants.PID_COEFS = new PIDFCoefs(1, 1, 1, 1, 0, 0);

        // Shooter Constants
        shooterConstants.CAN_MAP = can.shooterMap;
        shooterConstants.RIGHT_MOTOR_CONFIG = new MotorConfig();
        shooterConstants.LEFT_MOTOR_CONFIG = new MotorConfig(shooterConstants.RIGHT_MOTOR_CONFIG, false, false);

        // LED constants
        ledConstants.PWM_MAP = pwm.ledMap;

        // Intake constants
        intakeConstants.CAN_MAP = can.intakeMap;
        intakeConstants.MOTOR_CONFIG = new MotorConfig();
        intakeConstants.DEFAULT_MOTOR_POWER = 0.5;
        intakeConstants.STALL_CHECK_DELAY = 0.5;
        intakeConstants.STALL_CURRENT_LIMIT = 10;

        // Left climber constants
        leftClimberConstants.PWM_MAP = pwm.leftClimberMap;
        leftClimberConstants.IS_INVERTED = false;

        // Right climber constants
        rightClimberConstants.PWM_MAP = pwm.rightClimberMap;
        rightClimberConstants.IS_INVERTED = false;

        /** Robot Map **/

        // CAN

        // shooter map
        // can.shooterMap.RIGHT_MOTOR = new TrigonTalonFX(12,
        // shooterConstants.RIGHT_MOTOR_CONFIG);
        // can.shooterMap.LEFT_MOTOR = new TrigonTalonFX(13,
        // shooterConstants.LEFT_MOTOR_CONFIG);
        // can.intakeMap.MOTOR = new TrigonTalonSRX(14, intakeConstants.MOTOR_CONFIG);
        // can.triggerMap.MOTOR = new TrigonTalonSRX(15, triggerConstants.MOTOR_CONFIG,
        // triggerConstants.PID_COEFS);

        // Drivetrain map

        final FeedforwardConstants frontRightAngleConstants = new FeedforwardConstants(0.0036, 1.229, 1, 0.5, 60, 10, 17);
        final FeedforwardConstants frontLeftAngleConstants = new FeedforwardConstants(0.004, 1.233, frontRightAngleConstants);
        final FeedforwardConstants rearRightAngleConstants = new FeedforwardConstants(0.004, 1.2458, frontRightAngleConstants);
        final FeedforwardConstants rearLeftAngleConstants = new FeedforwardConstants(0.0036, 1.2501, frontRightAngleConstants);

        final FeedforwardConstants frontRightSpeedConstants = new FeedforwardConstants(1, 1, 1, 1, 0.2, 50, 10);
        final FeedforwardConstants frontLeftSpeedConstants = new FeedforwardConstants(1, 1, frontRightSpeedConstants);
        final FeedforwardConstants rearRightSpeedConstants = new FeedforwardConstants(1, 1, frontRightSpeedConstants);
        final FeedforwardConstants rearLeftSpeedConstants = new FeedforwardConstants(1, 1, frontRightSpeedConstants);
        

        drivetrainConstants.SPEED_PIDF_COEFS = new PIDFCoefs(0.000797, 0, 0);
        drivetrainConstants.ANGLE_PIDF_COEFS = new PIDFCoefs(0.17, 0.15, 0.000002, 0, 0, new TrapezoidProfile.Constraints(3000, 5000));

        drivetrainConstants.FRONT_RIGHT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(0, new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, true)),
                new TalonFXWithTalonSRXEncoder(1, 8,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                false)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.FRONT_RIGHT_LOCATION.getRotation().getDegrees(),
                drivetrainConstants.MAX_SPEED_MPS,
                drivetrainConstants.ANGLE_PIDF_COEFS,
                drivetrainConstants.SPEED_PIDF_COEFS,
                frontRightAngleConstants,
                frontRightSpeedConstants
        );
        drivetrainConstants.FRONT_LEFT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(2,
                        new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, false)),
                new TalonFXWithTalonSRXEncoder(3, 9,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                false)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.FRONT_LEFT_LOCATION.getRotation().getDegrees(),
                drivetrainConstants.MAX_SPEED_MPS,
                drivetrainConstants.ANGLE_PIDF_COEFS,
                drivetrainConstants.SPEED_PIDF_COEFS,
                frontLeftAngleConstants,
                frontLeftSpeedConstants
        );
        drivetrainConstants.REAR_RIGHT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(4, new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, true)),
                new TalonFXWithTalonSRXEncoder(5, 10,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                false)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.REAR_RIGHT_LOCATION.getRotation().getDegrees(),
                drivetrainConstants.MAX_SPEED_MPS,
                drivetrainConstants.ANGLE_PIDF_COEFS,
                drivetrainConstants.SPEED_PIDF_COEFS,
                rearRightAngleConstants,
                rearRightSpeedConstants
        );
        drivetrainConstants.REAR_LEFT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(6,
                        new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, false)),
                new TalonFXWithTalonSRXEncoder(7, 11,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                false)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.REAR_LEFT_LOCATION.getRotation().getDegrees(),
                drivetrainConstants.MAX_SPEED_MPS,
                drivetrainConstants.ANGLE_PIDF_COEFS,
                drivetrainConstants.SPEED_PIDF_COEFS,
                rearLeftAngleConstants,
                rearLeftSpeedConstants
        );

        can.drivetrainMap.FRONT_RIGHT = new SwerveModule(drivetrainConstants.FRONT_RIGHT_CONSTANTS);
        can.drivetrainMap.FRONT_LEFT = new SwerveModule(drivetrainConstants.FRONT_LEFT_CONSTANTS);
        can.drivetrainMap.REAR_RIGHT = new SwerveModule(drivetrainConstants.REAR_RIGHT_CONSTANTS);
        can.drivetrainMap.REAR_LEFT = new SwerveModule(drivetrainConstants.REAR_LEFT_CONSTANTS);

        can.drivetrainMap.GYRO = new Pigeon(new TrigonTalonSRX(12));

        // PWM
        pwm.ledMap.LED_CONTROLLER = 0;
        pwm.leftClimberMap.MOTOR = new PWMSparkMax(1);
        pwm.rightClimberMap.MOTOR = new PWMSparkMax(2);
    }
}
