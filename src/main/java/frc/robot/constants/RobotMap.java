package frc.robot.constants;

import edu.wpi.first.wpilibj.PWMSparkMax;
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

    // TODO: Set variables for hardware components

    public class CAN {
        public LoaderMap triggerMap = new LoaderMap();
        public ShooterMap shooterMap = new ShooterMap();
        public IntakeMap intakeMap = new IntakeMap();

        public class LoaderMap {
            public TrigonTalonSRX MOTOR;
        }

        public class ShooterMap {
            public TrigonTalonFX RIGHT_MOTOR;
            public TrigonTalonFX LEFT_MOTOR;
        }

        public class IntakeMap {
            public TrigonTalonSRX MOTOR;
        }
    }

    public class PCM {

    }

    public class DIO {

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
}
