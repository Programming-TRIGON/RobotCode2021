package frc.robot.constants.robots;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.components.*;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.PIDCoefs;
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
        drivetrainConstants.FRONT_LEFT_LOCATION = new Pose2d(0.29765, 0.29765, Rotation2d.fromDegrees(0));
        drivetrainConstants.FRONT_RIGHT_LOCATION = new Pose2d(0.29765, -0.29765, Rotation2d.fromDegrees(0));
        drivetrainConstants.REAR_LEFT_LOCATION = new Pose2d(-0.29765, 0.29765, Rotation2d.fromDegrees(0));
        drivetrainConstants.REAR_RIGHT_LOCATION = new Pose2d(-0.29765, -0.29765, Rotation2d.fromDegrees(0));
        drivetrainConstants.WHEEL_DIAMETER_M = 0.05; // in meters
        drivetrainConstants.MAX_SPEED_MPS = 5; // in m/s
        drivetrainConstants.MAX_ROT_SPEED_RAD_S = 3; //in rad/s

        StaticSwerveConstants.ANGLE_TICKS_PER_REVOLUTION = 4096;
        StaticSwerveConstants.SPEED_MOTOR_TICKS_PER_REVOLUTION = 2048;
        StaticSwerveConstants.ANGLE_DEFAULT_CONFIG = new MotorConfig(0.5, NeutralMode.Coast, 0);
        StaticSwerveConstants.SPEED_DEFAULT_CONFIG = new MotorConfig(0.5, NeutralMode.Coast, 0);
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
        visionConstants.ROTATION_SETTINGS = new PIDCoefs(0, 0, 0, 0, 0);
        visionConstants.TARGET_TIME_OUT = 0.1;

        // Loader Constants
        loaderConstants.CAN_MAP = can.loaderMap;
        loaderConstants.MOTOR_CONFIG = new MotorConfig();
        loaderConstants.PID_COEFS = new PIDCoefs(1, 1, 1, 1, 0, 0);
        loaderConstants.DEFAULT_SHOOTING_VELOCITY = 2000;
        loaderConstants.DEFAULT_MIXING_VELOCITY = -500;

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

        // Intake opener constants
        intakeOpenerConstants.CAN_MAP = can.intakeOpenerMap;
        intakeOpenerConstants.DIO_MAP = dio.intakeOpenerMap;
        intakeOpenerConstants.MOTOR_CONFIG = new MotorConfig();
        intakeOpenerConstants.LOGGABLE_NAME = "Intake";

        /** Robot Map **/

        // CAN

        can.shooterMap.RIGHT_MOTOR = new TrigonTalonFX(12, shooterConstants.RIGHT_MOTOR_CONFIG);
        can.shooterMap.LEFT_MOTOR = new TrigonTalonFX(13, shooterConstants.LEFT_MOTOR_CONFIG);
        can.intakeMap.MOTOR = new TrigonTalonSRX(8, intakeConstants.MOTOR_CONFIG);
        can.intakeOpenerMap.MOTOR = new TrigonTalonSRX(10, intakeOpenerConstants.MOTOR_CONFIG);
        can.loaderMap.MOTOR = new TrigonTalonSRX(15, loaderConstants.MOTOR_CONFIG, loaderConstants.PID_COEFS);

        // Drivetrain map;
        drivetrainConstants.FRONT_RIGHT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(0, new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, true, true)),
                new TalonFXWithTalonSRXEncoder(1, can.intakeMap.MOTOR,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true, true)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.FRONT_RIGHT_LOCATION.getRotation().getDegrees(),
                new PIDCoefs(1, 1, 1, 1, 1),
                new PIDCoefs(1, 1, 1, 1, 1)
        );
        drivetrainConstants.FRONT_LEFT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(2, new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, false, true)),
                new TalonFXWithTalonSRXEncoder(3, can.loaderMap.MOTOR,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true, true)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.FRONT_LEFT_LOCATION.getRotation().getDegrees(),
                new PIDCoefs(1, 1, 1, 1, 1),
                new PIDCoefs(1, 1, 1, 1, 1)
        );
        drivetrainConstants.REAR_RIGHT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(4, new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, true, true)),
                new TalonFXWithTalonSRXEncoder(5, can.intakeOpenerMap.MOTOR,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true, true)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.REAR_RIGHT_LOCATION.getRotation().getDegrees(),
                new PIDCoefs(1, 1, 1, 1, 1),
                new PIDCoefs(1, 1, 1, 1, 1)
        );
        drivetrainConstants.REAR_LEFT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(6, new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, false, true)),
                new TalonFXWithTalonSRXEncoder(7, 11,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true, true)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.REAR_LEFT_LOCATION.getRotation().getDegrees(),
                new PIDCoefs(1, 1, 1, 1, 1),
                new PIDCoefs(1, 1, 1, 1, 1)
        );

        can.drivetrainMap.FRONT_RIGHT = new SwerveModule(drivetrainConstants.FRONT_RIGHT_CONSTANTS);
        can.drivetrainMap.FRONT_LEFT = new SwerveModule(drivetrainConstants.FRONT_LEFT_CONSTANTS);
        can.drivetrainMap.REAR_RIGHT = new SwerveModule(drivetrainConstants.REAR_RIGHT_CONSTANTS);
        can.drivetrainMap.REAR_LEFT = new SwerveModule(drivetrainConstants.REAR_LEFT_CONSTANTS);

        can.drivetrainMap.GYRO = new Pigeon(12);

        // PWM
        pwm.ledMap.LED_CONTROLLER = 0;
        pwm.leftClimberMap.MOTOR = new PWMSparkMax(1);
        pwm.rightClimberMap.MOTOR = new PWMSparkMax(2);

        // DIO
        intakeOpenerConstants.DIO_MAP.OPEN_SWITCH = new DigitalInput(0);
        intakeOpenerConstants.DIO_MAP.CLOSED_SWITCH = new DigitalInput(1);
    }
}
