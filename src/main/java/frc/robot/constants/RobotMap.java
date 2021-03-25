package frc.robot.constants;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.PWMSparkMax;
import frc.robot.components.TrigonDoubleSolenoid;
import frc.robot.components.Pigeon;
import frc.robot.components.SwerveModule;
import frc.robot.components.TrigonDoubleSolenoid;
import frc.robot.components.TrigonTalonFX;
import frc.robot.components.TrigonTalonSRX;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public abstract class RobotMap {
    public CAN can = new CAN();
    public PCM pcm = new PCM();
    public DIO dio = new DIO();
    public PWM pwm = new PWM();
    public I2C i2c = new I2C();

    // TODO: Set variables for hardware components

    public class CAN {
        public DrivetrainMap drivetrainMap = new DrivetrainMap();
        public LoaderMap loaderMap = new LoaderMap();
        public ShooterMap shooterMap = new ShooterMap();
        public IntakeMap intakeMap = new IntakeMap();
        public IntakeOpenerMap intakeOpenerMap = new IntakeOpenerMap();
        public SpinnerMap spinnerMap = new SpinnerMap();

        public class LoaderMap {
            public TrigonTalonSRX MOTOR;
        }

        public class DrivetrainMap {
            public SwerveModule FRONT_RIGHT, FRONT_LEFT, REAR_RIGHT, REAR_LEFT;
            public Pigeon GYRO;
        }

        public class ShooterMap {
            public TrigonTalonFX RIGHT_MOTOR;
            public TrigonTalonFX LEFT_MOTOR;
        }

        public class IntakeMap {
            public TrigonTalonSRX MOTOR;
        }

        public class IntakeOpenerMap {
            public TrigonTalonSRX MOTOR;
        }

        public class SpinnerMap {
            public TrigonTalonSRX MOTOR;
            public ColorSensorV3 COLOR_SENSOR;
        }
    }

    public class PWM {
        public LedMap ledMap = new LedMap();
        public ClimberMap leftClimberMap = new ClimberMap();
        public ClimberMap rightClimberMap = new ClimberMap();

        public class LedMap {
            public int LED_CONTROLLER;
        }

        public class ClimberMap {
            public PWMSparkMax MOTOR;
        }

    }

    public class I2C {
        public SpinnerMap spinnerMap = new SpinnerMap();

        public class SpinnerMap {
            public ColorSensorV3 COLOR_SENSOR;
        }
    }

    public class PCM {
        public PitcherMap pitcherMap = new PitcherMap();
        public SpinnerMap spinnerMap = new SpinnerMap();

        public class PitcherMap {
            public TrigonDoubleSolenoid SOLENOID;
        }

        public class SpinnerMap {
            public TrigonDoubleSolenoid SOLENOID;
        }
    }

    public class DIO {
        public IntakeOpenerMap intakeOpenerMap = new IntakeOpenerMap();

        public class IntakeOpenerMap {
            public DigitalInput CLOSED_SWITCH;
            public DigitalInput OPEN_SWITCH;
        }
    }
}
