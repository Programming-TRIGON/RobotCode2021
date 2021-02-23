package frc.robot.constants.robots;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.components.*;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.PIDCoefs;
import frc.robot.utilities.SwerveConstants;

/**
 * instantiates the robot constants
 */
public class RobotA extends RobotConstants {

    // TODO: Set Constants
    public RobotA() {
        /* Robot constants */

        // Drivetrain constants
        drivetrainConstants.canDrivetrainMap = can.drivetrainMap;
        drivetrainConstants.FRONT_LEFT_LOCATION = new Pose2d(0.29765, 0.29765, Rotation2d.fromDegrees(0));
        drivetrainConstants.FRONT_RIGHT_LOCATION = new Pose2d(0.29765, -0.29765, Rotation2d.fromDegrees(0));
        drivetrainConstants.REAR_LEFT_LOCATION = new Pose2d(-0.29765, 0.29765, Rotation2d.fromDegrees(0));
        drivetrainConstants.REAR_RIGHT_LOCATION = new Pose2d(-0.29765, -0.29765, Rotation2d.fromDegrees(0));
        drivetrainConstants.WHEEL_DIAMETER_M = 0.05; // in meters
        drivetrainConstants.MAX_SPEED_MPS = 5; // in m/s
        drivetrainConstants.MAX_ROT_SPEED_RAD_S = 3; //in rad/s
        drivetrainConstants.SPEED_MOTOR_CONFIG = new MotorConfig();
        drivetrainConstants.ANGLE_MOTOR_CONFIG = new MotorConfig();

        SwerveConstants.StaticSwerveConstants.SPEED_MOTOR_TICKS_PER_REVOLUTION = 2048;
        SwerveConstants.StaticSwerveConstants.ANGLE_TICKS_PER_REVOLUTION = 4096;
        SwerveConstants.StaticSwerveConstants.SPEED_GEAR_RATION = 6.86;
        SwerveConstants.StaticSwerveConstants.DEFAULT_CONFIG = new MotorConfig(4, NeutralMode.Coast, 0);

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

        // Trigger Constants
        triggerConstants.CAN_MAP = can.triggerMap;
        triggerConstants.MOTOR_CONFIG = new MotorConfig();
        triggerConstants.PID_COEFS = new PIDCoefs(1, 1, 1, 1, 0, 0);

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

        // Spinner constants
        spinnerConstants.CAN_MAP = can.spinnerMap;
        spinnerConstants.PCM_MAP = pcm.spinnerMap;
        spinnerConstants.I2C_MAP = i2c.spinnerMap;
        spinnerConstants.MOTOR_CONFIG = new MotorConfig();


        /** Robot Map **/

        // CAN

        // shooter map
        can.shooterMap.RIGHT_MOTOR = new TrigonTalonFX(12, shooterConstants.RIGHT_MOTOR_CONFIG);
        can.shooterMap.LEFT_MOTOR = new TrigonTalonFX(13, shooterConstants.LEFT_MOTOR_CONFIG);
        can.intakeMap.MOTOR = new TrigonTalonSRX(8, intakeConstants.MOTOR_CONFIG);
        can.triggerMap.MOTOR = new TrigonTalonSRX(9, triggerConstants.MOTOR_CONFIG, triggerConstants.PID_COEFS);
        can.spinnerMap.MOTOR = new TrigonTalonSRX(3, spinnerConstants.MOTOR_CONFIG);

        // Drivetrain map;
        drivetrainConstants.FRONT_RIGHT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(0),
                new TalonFXWithTalonSRXEncoder(1, 8,
                        new MotorConfig(SwerveConstants.StaticSwerveConstants.DEFAULT_CONFIG, false, false)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.FRONT_RIGHT_LOCATION.getRotation().getDegrees(),
                new PIDCoefs(1, 1, 1, 1, 1),
                new PIDCoefs(1, 1, 1, 1, 1)
        );
        drivetrainConstants.FRONT_LEFT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(2),
                new TalonFXWithTalonSRXEncoder(3, 9,
                        new MotorConfig(SwerveConstants.StaticSwerveConstants.DEFAULT_CONFIG, false, false)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.FRONT_LEFT_LOCATION.getRotation().getDegrees(),
                new PIDCoefs(1, 1, 1, 1, 1),
                new PIDCoefs(1, 1, 1, 1, 1)
        );
        drivetrainConstants.REAR_RIGHT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(4),
                new TalonFXWithTalonSRXEncoder(5, 10,
                        new MotorConfig(SwerveConstants.StaticSwerveConstants.DEFAULT_CONFIG, false, false)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.REAR_RIGHT_LOCATION.getRotation().getDegrees(),
                new PIDCoefs(1, 1, 1, 1, 1),
                new PIDCoefs(1, 1, 1, 1, 1)
        );
        drivetrainConstants.REAR_LEFT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(6),
                new TalonFXWithTalonSRXEncoder(7, 11,
                        new MotorConfig(SwerveConstants.StaticSwerveConstants.DEFAULT_CONFIG, false, false)),
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

        // PCM
        pcm.spinnerMap.SOLENOID = new TrigonDoubleSolenoid(0, 1);

        // I2C
        i2c.spinnerMap.COLOR_SENSOR = new ColorSensorV3(Port.kOnboard);
    }
}
