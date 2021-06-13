package frc.robot.constants.robots;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import frc.robot.components.*;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.*;
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
        drivetrainConstants.FRONT_LEFT_LOCATION = new Pose2d(0.29765, -0.29765, Rotation2d.fromDegrees(213.57 + 100 - 29 - 90));
        drivetrainConstants.FRONT_RIGHT_LOCATION = new Pose2d(0.29765, 0.29765, Rotation2d.fromDegrees(109.6 - 90 - 90));
        drivetrainConstants.REAR_LEFT_LOCATION = new Pose2d(-0.29765, -0.29765, Rotation2d.fromDegrees(292.41 + 77 - 90));
        drivetrainConstants.REAR_RIGHT_LOCATION = new Pose2d(-0.29765, 0.29765, Rotation2d.fromDegrees(157.41 + 80 - 90));
        drivetrainConstants.WHEEL_DIAMETER_M = 0.1016; // in meters
        drivetrainConstants.MAX_SPEED_MPS = 10; // in m/s
        drivetrainConstants.MAX_ROT_SPEED_RAD_S = 15; // in rad/s

        StaticSwerveConstants.ANGLE_TICKS_PER_REVOLUTION = 4096;
        StaticSwerveConstants.SPEED_MOTOR_TICKS_PER_REVOLUTION = 2048;
        StaticSwerveConstants.ANGLE_DEFAULT_CONFIG = new MotorConfig(.1, false, false, NeutralMode.Brake, 0, new SupplyCurrentLimitConfiguration(true, 10, .5, .2));
        StaticSwerveConstants.SPEED_DEFAULT_CONFIG = new MotorConfig(.5, false, false, NeutralMode.Brake, 0, new SupplyCurrentLimitConfiguration(true, 12, .5, .2));
        StaticSwerveConstants.SPEED_GEAR_RATION = 6.86;

        // Sensor check constants
        testerConstants.MOVE_POWER = 1;
        testerConstants.SECONDS_TO_WAIT = 3;

        // Vision Constants
        visionConstants.ROTATION_SETTINGS = new PIDFCoefs(0.005, 0, 0.0, 0.08, 0.001);
        visionConstants.POSITION_SETTINGS = new PIDFCoefs(0.05, 0, 0.0, 0.16, 0.05);
        visionConstants.Y_TARGET = 3.75;
        visionConstants.TARGET_TIME_OUT = 4;

        // Loader Constants
        loaderConstants.CAN_MAP = can.loaderMap;
        loaderConstants.MOTOR_CONFIG = new MotorConfig(0.2, false, true, NeutralMode.Coast, 0);
        loaderConstants.FEEDFORWARD_CONSTANTS = new FeedforwardConstants(0.082812504229243, 0, 0.1, 0.1, 5000, 150, 8);
        loaderConstants.PID_COEFS = new PIDFCoefs(0, 0, 0, loaderConstants.FEEDFORWARD_CONSTANTS.mCoef, 0, 0);
        loaderConstants.DEFAULT_SHOOTING_VELOCITY = 11500;
        loaderConstants.DEFAULT_MIXING_VELOCITY = -1500;

        // Shooter Constants
        shooterConstants.CAN_MAP = can.shooterMap;
        shooterConstants.RIGHT_MOTOR_CONFIG = new MotorConfig(3, false, false, NeutralMode.Coast, 0);
        shooterConstants.LEFT_MOTOR_CONFIG = new MotorConfig(shooterConstants.RIGHT_MOTOR_CONFIG, true, false);
        /*
         * Tolerance and delta tolerance in the PIDCoefs are for deciding when to change
         * to TBH and the tolerance and delta tolerance constants are for deciding when
         * we are ready to shoot
         */
        shooterConstants.PID_COEFS = new PIDFCoefs(0.0016, 0.00002, 0.000, 30, 0);
        shooterConstants.TBH_CONTROLLER = new TBHController(0.00005, shooterConstants.PID_COEFS.getTolerance());
        shooterConstants.PID_CONTROLLER = new TrigonPIDController(shooterConstants.PID_COEFS);
        shooterConstants.SIMPLE_MOTOR_FEEDFORWARD = new SimpleMotorFeedforward(0.812, 0.140, 0.00984);
        shooterConstants.KF_COEF_A = 0.0019;
        shooterConstants.KF_COEF_B = 0.8085;
        shooterConstants.SHOOTING_RAMP_RATE = .7;
        //TODO: reset to normal value
        shooterConstants.TOLERANCE = 250;
        shooterConstants.TIME_AT_SETPOINT = 0.25;
        shooterConstants.CANCEL_CMDGP_AXIS_THRESHOLD = 0.4;
        shooterConstants.MAX_NUMBER_OF_BALLS = 5;
        shooterConstants.KF_CALCULATION_SAMPLE_AMOUNT = 40;
        shooterConstants.KF_TESTING_DELTA_TOLERANCE = 5;
        shooterConstants.KF_TESTING_TOLERANCE = 10;
        shooterConstants.KF_TESTING_INITIAL_DESIRED_VELOCITY = 300;
        shooterConstants.KF_TESTING_VELOCITY_ACCELERATION_PER_TEST = 200;
        shooterConstants.KF_TESTING_TEST_AMOUNT = 20;
        shooterConstants.KF_TESTING_CALCULATION_SAMPLE_AMOUNT = 100;
        shooterConstants.AREA_ARRAY = new int[]{3170, 3060, 3500, 2050};


        // LED constants
        ledConstants.PWM_MAP = pwm.ledMap;

        // Intake constants
        intakeConstants.CAN_MAP = can.intakeMap;
        intakeConstants.MOTOR_CONFIG = new MotorConfig();
        intakeConstants.DEFAULT_MOTOR_POWER = 0.5;

        // Left climber constants
        leftClimberConstants.PWM_MAP = pwm.leftClimberMap;
        leftClimberConstants.IS_INVERTED = false;

        // Right climber constants
        rightClimberConstants.PWM_MAP = pwm.rightClimberMap;
        rightClimberConstants.IS_INVERTED = false;

        // Motion profile constants
        motionProfilingConstants.MAX_VELOCITY = 5;
        motionProfilingConstants.MAX_ACCELERATION = 1;
        motionProfilingConstants.MAX_CENTRIPETAL_ACCELERATION = 0.5;
        motionProfilingConstants.KP = 0;
        motionProfilingConstants.REVERSE_KP = 0;
        motionProfilingConstants.X_PID_CONTROLLER = new TrigonPIDController(new PIDFCoefs(0, 0, 0));
        motionProfilingConstants.Y_PID_CONTROLLER = new TrigonPIDController(new PIDFCoefs(0, 0, 0));
        motionProfilingConstants.THETA_PROFILED_PID_CONTROLLER = new TrigonProfiledPIDController(
                new PIDFCoefs(0.5, 0, 0, 0, 0, new Constraints(100, 50)));
        // Pitcher constants
        pitcherConstants.PCM_MAP = pcm.pitcherMap;
        pitcherConstants.EXTENDED_TOGGLE_ANGLE = 20;
        pitcherConstants.RETRACTED_TOGGLE_ANGLE = 10;
        pitcherConstants.NO_TARGET_BLINK_TIME = 5;

        // Intake opener constants
        intakeOpenerConstants.PCM_MAP = pcm.intakeOpenerMap;

        // Spinner constants
        spinnerConstants.CAN_MAP = can.spinnerMap;
        spinnerConstants.I2C_MAP = i2c.spinnerMap;
        spinnerConstants.MOTOR_CONFIG = new MotorConfig(0.5, NeutralMode.Coast, 0);
        spinnerConstants.DEFAULT_MOTOR_POWER = -0.15;
        spinnerConstants.STALL_CURRENT_LIMIT = 20;
        spinnerConstants.STALL_CHECK_DELAY = 2;

        /* Limelight Constants */

        // extended limelight
        extendedLimelightConstants.DEFAULT_TABLE_KEY = "limelight";
        extendedLimelightConstants.LIMELIGHT_OFFSET_X = 0;
        extendedLimelightConstants.LIMELIGHT_OFFSET_Y = 0;
        extendedLimelightConstants.LIMELIGHT_ANGLE_OFFSET = 0;
        extendedLimelightConstants.DISTANCE_CALCULATION_A_COEFFICIENT = 1;
        extendedLimelightConstants.DISTANCE_CALCULATION_B_COEFFICIENT = 1;
        extendedLimelightConstants.DISTANCE_CALCULATION_C_COEFFICIENT = 1;
        extendedLimelightConstants.SHOOTER_DISTANCE_TO_VELOCITY_COEF_A = 0;
        extendedLimelightConstants.SHOOTER_DISTANCE_TO_VELOCITY_COEF_B = 156;
        extendedLimelightConstants.SHOOTER_DISTANCE_TO_VELOCITY_COEF_C = 2656;
        extendedLimelightConstants.SHOOTER_HEIGHT_TO_DISTANCE_COEF_A = 1.69 * Math.pow(10, -5);
        extendedLimelightConstants.SHOOTER_HEIGHT_TO_DISTANCE_COEF_B = -7.04 * Math.pow(10, -4);
        extendedLimelightConstants.SHOOTER_HEIGHT_TO_DISTANCE_COEF_C = 0.0137;

        // retracted limelight
        retractedLimelightConstants.DEFAULT_TABLE_KEY = extendedLimelightConstants.DEFAULT_TABLE_KEY;
        retractedLimelightConstants.LIMELIGHT_OFFSET_X = 0;
        retractedLimelightConstants.LIMELIGHT_OFFSET_Y = 0;
        retractedLimelightConstants.LIMELIGHT_ANGLE_OFFSET = 0;
        retractedLimelightConstants.DISTANCE_CALCULATION_A_COEFFICIENT = -17;
        retractedLimelightConstants.DISTANCE_CALCULATION_B_COEFFICIENT = 0;
        retractedLimelightConstants.DISTANCE_CALCULATION_C_COEFFICIENT = 24.1;
        retractedLimelightConstants.SHOOTER_DISTANCE_TO_VELOCITY_COEF_A = 0;
        retractedLimelightConstants.SHOOTER_DISTANCE_TO_VELOCITY_COEF_B = 156;
        retractedLimelightConstants.SHOOTER_DISTANCE_TO_VELOCITY_COEF_C = 2656;
        retractedLimelightConstants.SHOOTER_HEIGHT_TO_DISTANCE_COEF_A =  1.69 * Math.pow(10, -5);
        retractedLimelightConstants.SHOOTER_HEIGHT_TO_DISTANCE_COEF_B = -7.04 * Math.pow(10, -4);
        retractedLimelightConstants.SHOOTER_HEIGHT_TO_DISTANCE_COEF_C = 0.0137;

        /* Robot Map */

        // CAN

        can.spinnerMap.MOTOR = new TrigonTalonSRX(11, spinnerConstants.MOTOR_CONFIG);
        can.shooterMap.RIGHT_MOTOR = new TrigonTalonFX(13, shooterConstants.RIGHT_MOTOR_CONFIG);
        can.shooterMap.LEFT_MOTOR = new TrigonTalonFX(14, shooterConstants.LEFT_MOTOR_CONFIG);
        can.intakeMap.MOTOR = new TrigonTalonSRX(9, intakeConstants.MOTOR_CONFIG);
        can.loaderMap.MOTOR = new TrigonTalonSRX(17, loaderConstants.MOTOR_CONFIG, loaderConstants.PID_COEFS);

        // Drivetrain map;
        drivetrainConstants.SPEED_SVA_COEFS = new SVACoefs(0.557, 2.35, 0.0749);
        drivetrainConstants.SPEED_PIDF_COEFS = new PIDFCoefs(0.000797, 0, 0);
//        drivetrainConstants.SPEED_SVA_COEFS = new SVACoefs(0, 0, 0);
//        drivetrainConstants.SPEED_PIDF_COEFS = new PIDFCoefs(0, new TrapezoidProfile.Constraints(0, 0));

        drivetrainConstants.ANGLE_SVA_COEFS = new SVACoefs(0.7, 0.00384, 4.36e-5);
        drivetrainConstants.ANGLE_PIDF_COEFS = new PIDFCoefs(0.09, new TrapezoidProfile.Constraints(15000, 10000));
        drivetrainConstants.ROTATION_PIDF_COEFS = new PIDFCoefs(0.005, 0, 0, 3, 20);

        drivetrainConstants.FRONT_RIGHT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(0, new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, false)),
                new TalonFXWithTalonSRXEncoder(1, 8,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                false)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.FRONT_RIGHT_LOCATION.getRotation().getDegrees(),
                drivetrainConstants.MAX_SPEED_MPS,
                new PIDFCoefs(0.04, 0.4, 0.0003, 1, 1, new TrapezoidProfile.Constraints(15000, 10000)),
                new PIDFCoefs(0.731, 0, 0),
                new SVACoefs(0.742, 0.00408, 9.46e-5),
                new SVACoefs(0.719, 2.39, 0.0718)
        );
        drivetrainConstants.FRONT_LEFT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(2,
                        new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, false)),
                new TalonFXWithTalonSRXEncoder(3, can.intakeMap.MOTOR,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                false)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.FRONT_LEFT_LOCATION.getRotation().getDegrees(),
                drivetrainConstants.MAX_SPEED_MPS,
                new PIDFCoefs(0.04, 0.5, 0.0003, 1, 1, new TrapezoidProfile.Constraints(15000, 10000)),
                new PIDFCoefs(0.616, 0, 0),
                new SVACoefs(0.748, 0.00403, 0.000122),
                new SVACoefs(0.699, 2.46, 0.0355)
        );
        drivetrainConstants.REAR_RIGHT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(4, new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, true)),
                new TalonFXWithTalonSRXEncoder(5, 10,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                false)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.REAR_RIGHT_LOCATION.getRotation().getDegrees(),
                drivetrainConstants.MAX_SPEED_MPS,
                new PIDFCoefs(0.02, 0.7, 0.0003, 1, 1, new TrapezoidProfile.Constraints(15000, 10000)),
                new PIDFCoefs(2.32, 0, 0),
                new SVACoefs(0.785, 0.00417, 0.00011),
                new SVACoefs(0.643, 2.37, 0.361)
        );
        drivetrainConstants.REAR_LEFT_CONSTANTS = new SwerveConstants(
                new TrigonTalonFX(6,
                        new MotorConfig(StaticSwerveConstants.SPEED_DEFAULT_CONFIG, false)),
                new TalonFXWithTalonSRXEncoder(7, can.spinnerMap.MOTOR,
                        new MotorConfig(StaticSwerveConstants.ANGLE_DEFAULT_CONFIG, true,
                                false)),
                drivetrainConstants.WHEEL_DIAMETER_M,
                drivetrainConstants.REAR_LEFT_LOCATION.getRotation().getDegrees(),
                drivetrainConstants.MAX_SPEED_MPS,
                new PIDFCoefs(0.03, 0.6, 0.0003, 1, 1, new TrapezoidProfile.Constraints(15000, 10000)),
                new PIDFCoefs(2.48, 0, 0),
                new SVACoefs(0.786, 0.00411, 0.000106),
                new SVACoefs(0.65, 2.3, 0.396)
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

        // PCM
        pcm.compressorMap.COMPRESSOR = new Compressor(0);
        pcm.pitcherMap.SOLENOID = new TrigonDoubleSolenoid(1, 0);
        //pcm.spinnerMap.SOLENOID = new TrigonDoubleSolenoid(2, 3);
        pcm.intakeOpenerMap.SOLENOID = new TrigonDoubleSolenoid(6, 7);

        // I2C
        i2c.spinnerMap.COLOR_SENSOR = new ColorSensorV3(Port.kOnboard);
    }
}

