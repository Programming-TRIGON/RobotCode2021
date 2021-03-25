package frc.robot.constants.robots;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.commands.GenericCalibrateKF;
import frc.robot.components.*;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.FeedforwardConstants;
import frc.robot.utilities.PIDCoefs;
import frc.robot.utilities.SwerveConstants;
import frc.robot.utilities.SwerveConstants.StaticSwerveConstants;
import frc.robot.utilities.TrigonPIDController;

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
        drivetrainConstants.MAX_ROT_SPEED_RAD_S = 3; // in rad/s

        StaticSwerveConstants.ANGLE_TICKS_PER_REVOLUTION = 4096;
        StaticSwerveConstants.SPEED_MOTOR_TICKS_PER_REVOLUTION = 2048;
        StaticSwerveConstants.ANGLE_DEFAULT_CONFIG = new MotorConfig(0.5, NeutralMode.Coast, 0);
        StaticSwerveConstants.SPEED_DEFAULT_CONFIG = new MotorConfig(0.5, NeutralMode.Coast, 0);
        StaticSwerveConstants.SPEED_GEAR_RATION = 6.86;

        // Sensor check constants
        testerConstants.MOVE_POWER = 1;
        testerConstants.SECONDS_TO_WAIT = 3;

        // Vision Constants
        visionConstants.ROTATION_SETTINGS = new PIDCoefs(0, 0, 0, 0, 0);
        visionConstants.TARGET_TIME_OUT = 0.1;

        // Loader Constants
        loaderConstants.CAN_MAP = can.loaderMap;
        loaderConstants.MOTOR_CONFIG = new MotorConfig();
        loaderConstants.FEEDFORWARD_CONSTANTS = new FeedforwardConstants(0, 0, 0.1, 0.1, 0.1, 5, 8);
        loaderConstants.PID_COEFS = new PIDCoefs(1, 1, 1, loaderConstants.FEEDFORWARD_CONSTANTS.mCoef, 0, 0);
        loaderConstants.DEFAULT_SHOOTING_VELOCITY = 2000;
        loaderConstants.DEFAULT_MIXING_VELOCITY = -500;

        // Shooter Constants
        shooterConstants.CAN_MAP = can.shooterMap;
        shooterConstants.RIGHT_MOTOR_CONFIG = new MotorConfig(15, false, false, NeutralMode.Coast, 0);
        shooterConstants.LEFT_MOTOR_CONFIG = new MotorConfig(shooterConstants.RIGHT_MOTOR_CONFIG, true, false);
        /*
         * Tolerance and delta tolerance in the PIDCoefs are for deciding when to change
         * to TBH and the tolerance and delta tolerance constants are for deciding when
         * we are ready to shoot
         */
        shooterConstants.PID_COEFS = new PIDCoefs(0, 0, 0.000, 30, 0);
        shooterConstants.TBH_CONTROLLER = new TBHController(0.00005, shooterConstants.PID_COEFS.getTolerance());
        shooterConstants.PID_CONTROLLER = new TrigonPIDController(shooterConstants.PID_COEFS);
        shooterConstants.SIMPLE_MOTOR_FEEDFORWARD = new SimpleMotorFeedforward(0.812, 0.122, 0.00984);
        shooterConstants.LIMELIGHT_VELOCITY_COEF_A = 1;
        shooterConstants.LIMELIGHT_VELOCITY_COEF_B = 1;
        shooterConstants.LIMELIGHT_VELOCITY_COEF_C = 1;
        shooterConstants.KF_COEF_A = 0.0019;
        shooterConstants.KF_COEF_B = 0.7729;
        shooterConstants.SHOOTING_RAMP_RATE = 2;
        shooterConstants.TOLERANCE = 10;
        shooterConstants.DELTA_TOLERANCE = 4;
        shooterConstants.MAX_NUMBER_OF_BALLS = 5;
        shooterConstants.KF_CALCULATION_SAMPLE_AMOUNT = 30;
        shooterConstants.KF_TESTING_DELTA_TOLERANCE = 5;
        shooterConstants.KF_TESTING_TOLERANCE = 10;
        shooterConstants.KF_TESTING_INITIAL_DESIRED_VELOCITY = 100;
        shooterConstants.KF_TESTING_VELOCITY_ACCELERATION_PER_TEST = 200;
        shooterConstants.KF_TESTING_TEST_AMOUNT = 20;
        shooterConstants.KF_TESTING_CALCULATION_SAMPLE_AMOUNT = 100;

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

        // Pitcher constants
        pitcherConstants.PCM_MAP = pcm.pitcherMap;
        pitcherConstants.EXTENDED_ANGLE = 20;
        pitcherConstants.RETRACTED_ANGLE = 10;

        // Intake opener constants
        intakeOpenerConstants.CAN_MAP = can.intakeOpenerMap;
        intakeOpenerConstants.DIO_MAP = dio.intakeOpenerMap;
        intakeOpenerConstants.MOTOR_CONFIG = new MotorConfig();
        intakeOpenerConstants.LOGGABLE_NAME = "Intake";

        // Spinner constants
        spinnerConstants.CAN_MAP = can.spinnerMap;
        spinnerConstants.I2C_MAP = i2c.spinnerMap;
        spinnerConstants.MOTOR_CONFIG = new MotorConfig(5, NeutralMode.Coast, 0);
        spinnerConstants.DEFAULT_MOTOR_POWER = 0.5;
        intakeConstants.STALL_CHECK_DELAY = 0.5;
        spinnerConstants.STALL_CURRENT_LIMIT = 10;

        /* Limelight Constants */

        // vanilla limelight
        vanillaLimelightConstants.DEFAULT_TABLE_KEY = "Limelight";
        vanillaLimelightConstants.LIMELIGHT_OFFSET_X = 0;
        vanillaLimelightConstants.LIMELIGHT_OFFSET_Y = 0;
        vanillaLimelightConstants.LIMELIGHT_ANGLE_OFFSET = 0;
        vanillaLimelightConstants.DISTANCE_CALCULATION_A_COEFFICIENT = 1;
        vanillaLimelightConstants.DISTANCE_CALCULATION_B_COEFFICIENT = 1;
        vanillaLimelightConstants.DISTANCE_CALCULATION_C_COEFFICIENT = 1;

        // extended limelight
        extendedLimelightConstants.DEFAULT_TABLE_KEY = vanillaLimelightConstants.DEFAULT_TABLE_KEY;
        extendedLimelightConstants.LIMELIGHT_OFFSET_X = 0;
        extendedLimelightConstants.LIMELIGHT_OFFSET_Y = 0;
        extendedLimelightConstants.LIMELIGHT_ANGLE_OFFSET = 0;
        extendedLimelightConstants.DISTANCE_CALCULATION_A_COEFFICIENT = 1;
        extendedLimelightConstants.DISTANCE_CALCULATION_B_COEFFICIENT = 1;
        extendedLimelightConstants.DISTANCE_CALCULATION_C_COEFFICIENT = 1;

        // retracted limelight
        retractedLimelightConstants.DEFAULT_TABLE_KEY = vanillaLimelightConstants.DEFAULT_TABLE_KEY;
        retractedLimelightConstants.LIMELIGHT_OFFSET_X = 0;
        retractedLimelightConstants.LIMELIGHT_OFFSET_Y = 0;
        retractedLimelightConstants.LIMELIGHT_ANGLE_OFFSET = 0;
        retractedLimelightConstants.DISTANCE_CALCULATION_A_COEFFICIENT = 1;
        retractedLimelightConstants.DISTANCE_CALCULATION_B_COEFFICIENT = 1;
        retractedLimelightConstants.DISTANCE_CALCULATION_C_COEFFICIENT = 1;

        /** Robot Map **/

        // CAN

        can.shooterMap.RIGHT_MOTOR = new TrigonTalonFX(13, shooterConstants.RIGHT_MOTOR_CONFIG);
        can.shooterMap.LEFT_MOTOR = new TrigonTalonFX(14, shooterConstants.LEFT_MOTOR_CONFIG);
        can.intakeMap.MOTOR = new TrigonTalonSRX(15, intakeConstants.MOTOR_CONFIG);
        can.intakeOpenerMap.MOTOR = new TrigonTalonSRX(16, intakeOpenerConstants.MOTOR_CONFIG);
        can.loaderMap.MOTOR = new TrigonTalonSRX(17, loaderConstants.MOTOR_CONFIG, loaderConstants.PID_COEFS);
        can.spinnerMap.MOTOR = new TrigonTalonSRX(18, spinnerConstants.MOTOR_CONFIG);

        // Drivetrain map;
        drivetrainConstants.FRONT_RIGHT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(0,
                        new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, true,
                                true)),
                new TalonFXWithTalonSRXEncoder(1, can.intakeMap.MOTOR,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                true)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.FRONT_RIGHT_LOCATION.getRotation().getDegrees(),
                new PIDCoefs(1, 1, 1, 1, 1), new PIDCoefs(1, 1, 1, 1, 1));
        drivetrainConstants.FRONT_LEFT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(2,
                        new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, false,
                                true)),
                new TalonFXWithTalonSRXEncoder(3, can.loaderMap.MOTOR,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                true)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.FRONT_LEFT_LOCATION.getRotation().getDegrees(),
                new PIDCoefs(1, 1, 1, 1, 1), new PIDCoefs(1, 1, 1, 1, 1));
        drivetrainConstants.REAR_RIGHT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(4,
                        new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, true,
                                true)),
                new TalonFXWithTalonSRXEncoder(5, can.intakeOpenerMap.MOTOR,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                true)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.REAR_RIGHT_LOCATION.getRotation().getDegrees(),
                new PIDCoefs(1, 1, 1, 1, 1), new PIDCoefs(1, 1, 1, 1, 1));
        drivetrainConstants.REAR_LEFT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(6,
                        new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, false,
                                true)),
                new TalonFXWithTalonSRXEncoder(7, 11,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                true)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.REAR_LEFT_LOCATION.getRotation().getDegrees(),
                new PIDCoefs(1, 1, 1, 1, 1), new PIDCoefs(1, 1, 1, 1, 1));

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

        // PCM
        pcm.spinnerMap.SOLENOID = new TrigonDoubleSolenoid(0, 1);
        pcm.pitcherMap.SOLENOID = new TrigonDoubleSolenoid(2, 3);

        // I2C
        i2c.spinnerMap.COLOR_SENSOR = new ColorSensorV3(Port.kOnboard);
    }
}
