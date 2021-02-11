package frc.robot.constants;

import edu.wpi.first.wpilibj.PWMSparkMax;
import frc.robot.components.TrigonDoubleSolenoid;
import frc.robot.components.Pigeon;
import frc.robot.components.SwerveModule;
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
        public DrivetrainMap drivetrainMap = new DrivetrainMap();
        public TriggerMap triggerMap = new TriggerMap();
        public ShooterMap shooterMap = new ShooterMap();
        public IntakeMap intakeMap = new IntakeMap();

        public class TriggerMap {
            public TrigonTalonSRX MOTOR;
        }

        public class DrivetrainMap {
            public SwerveModule
                    FRONT_RIGHT,
                    FRONT_LEFT,
                    REAR_RIGHT,
                    REAR_LEFT;
            public Pigeon GYRO;
        }

        public class ShooterMap {
            public TrigonTalonFX RIGHT_MOTOR;
            public TrigonTalonFX LEFT_MOTOR;
        }

        public class IntakeMap {
            public TrigonTalonSRX MOTOR;
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

    public class PCM {
        public PitcherMap pitcherMap = new PitcherMap();

        public class PitcherMap {
            public TrigonDoubleSolenoid SOLENOID;
        }
    }

    public class DIO {

    }
}
