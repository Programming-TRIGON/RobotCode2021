package frc.robot.constants.robots;

import frc.robot.components.MotorConfig;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.PIDCoefs;

/**
 * instantiates the robot constants
 */
public class RobotA extends RobotConstants {

    // TODO: Set Constants
    public RobotA() {

        /* Robot Map */
        pwm.led.LED_CONTROLLER = 0;

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
        can.shooterMap.RIGHT_MOTOR_ID = 0;
        can.shooterMap.LEFT_MOTOR_ID = 1;
        shooterConstants.RIGHT_MOTOR_CONFIG = new MotorConfig();
        shooterConstants.LEFT_MOTOR_CONFIG = new MotorConfig(shooterConstants.RIGHT_MOTOR_CONFIG, false, false);
        shooterConstants.MILISECONDS_IN_MINUTE = 6000;
        shooterConstants.TICKS_PER_REVOLUTION = 4096;
    }
}
