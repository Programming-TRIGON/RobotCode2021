package frc.robot.constants.robots;


import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.components.MotorConfig;
import frc.robot.components.Pigeon;
import frc.robot.components.SwerveModule;
import frc.robot.components.TrigonTalonFX;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.PIDCoefs;

/**
 * instantiates the robot constants
 */
public class RobotA extends RobotConstants {

    // TODO: Set Constants
    public RobotA() {

        /* Robot constants */

        // LED constants
        ledConstants.LED_PWM_MAP = pwm.ledMap;

        // Drivetrain constants
        drivetrainConstants.canDrivetrainMap = can.drivetrainMap;
        drivetrainConstants.FRONT_LEFT_LOCATION = new Pose2d(0.381, 0.381, Rotation2d.fromDegrees(0));
        drivetrainConstants.FRONT_RIGHT_LOCATION = new Pose2d(0.381, -0.381, Rotation2d.fromDegrees(0));
        drivetrainConstants.REAR_LEFT_LOCATION = new Pose2d(-0.381, 0.381, Rotation2d.fromDegrees(0));
        drivetrainConstants.REAR_RIGHT_LOCATION = new Pose2d(-0.381, -0.381, Rotation2d.fromDegrees(0));
        drivetrainConstants.WHEEL_RADIUS = 0.05; // in meters
        drivetrainConstants.MAX_SPEED = 5; // in m/s
        drivetrainConstants.MAX_ROT_SPEED = 3; //in rad/s
        drivetrainConstants.SPEED_MOTOR_CONFIG = new MotorConfig();
        drivetrainConstants.ANGLE_MOTOR_CONFIG = new MotorConfig();

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
        testerConstants.LED_BLINK_AMOUNT = 10;

        // Vision Constants
        visionConstants.ROTATION_SETTINGS = new PIDCoefs(0, 0, 0, 0, 0);
        visionConstants.TARGET_TIME_OUT = 0.1;

        // Shooter Constants
        shooterConstants.canShooterMap = can.shooterMap;
        shooterConstants.RIGHT_MOTOR_CONFIG = new MotorConfig();
        shooterConstants.LEFT_MOTOR_CONFIG = new MotorConfig(shooterConstants.RIGHT_MOTOR_CONFIG, false, false);
        shooterConstants.CENTISECONDS_IN_MINUTE = 6000;
        shooterConstants.TICKS_PER_REVOLUTION = 4096;


        /* Robot Map */

        // led map
        pwm.led.LED_CONTROLLER = 0;

        // Shooter map
        can.shooterMap.RIGHT_MOTOR = new TrigonTalonFX(0, shooterConstants.RIGHT_MOTOR_CONFIG);
        can.shooterMap.LEFT_MOTOR = new TrigonTalonFX(1, shooterConstants.LEFT_MOTOR_CONFIG);


        // Drivetrain map;
        can.drivetrainMap.FRONT_RIGHT = new SwerveModule(0, 1, drivetrainConstants.WHEEL_RADIUS, drivetrainConstants.FRONT_RIGHT_LOCATION.getRotation().getDegrees());
        can.drivetrainMap.FRONT_LEFT = new SwerveModule(2, 3, drivetrainConstants.WHEEL_RADIUS, drivetrainConstants.FRONT_LEFT_LOCATION.getRotation().getDegrees());
        can.drivetrainMap.REAR_RIGHT = new SwerveModule(4, 5, drivetrainConstants.WHEEL_RADIUS, drivetrainConstants.REAR_RIGHT_LOCATION.getRotation().getDegrees());
        can.drivetrainMap.REAR_LEFT = new SwerveModule(6, 7, drivetrainConstants.WHEEL_RADIUS, drivetrainConstants.REAR_LEFT_LOCATION.getRotation().getDegrees());

        can.drivetrainMap.gyro = new Pigeon(0);

    }
}
