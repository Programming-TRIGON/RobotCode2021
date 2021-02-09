package frc.robot.constants.robots;

import edu.wpi.first.wpilibj.PWMSparkMax;
import frc.robot.components.MotorConfig;
import frc.robot.components.TrigonTalonFX;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.PIDCoefs;

/**
 * instantiates the robot constants
 */
public class RobotA extends RobotConstants {

    // TODO: Set Constants
    public RobotA() {
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

        /* Robot Map */

        // CAN
        can.shooterMap.RIGHT_MOTOR = new TrigonTalonFX(0, shooterConstants.RIGHT_MOTOR_CONFIG);
        can.shooterMap.LEFT_MOTOR = new TrigonTalonFX(1, shooterConstants.LEFT_MOTOR_CONFIG);
        can.intakeMap.MOTOR = new TrigonTalonSRX(2, intakeConstants.MOTOR_CONFIG);
        can.triggerMap.MOTOR = new TrigonTalonSRX(3, triggerConstants.MOTOR_CONFIG, triggerConstants.PID_COEFS);

        // PWM
        pwm.ledMap.LED_CONTROLLER = 0;
        pwm.leftClimberMap.MOTOR = new PWMSparkMax(1);
        pwm.rightClimberMap.MOTOR = new PWMSparkMax(2);
    }
}
