package frc.robot.constants;

import frc.robot.components.TrigonTalonFX;

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
        public ShooterMap shooterMap = new ShooterMap();

        public class ShooterMap {
            public TrigonTalonFX RIGHT_MOTOR;
            public TrigonTalonFX LEFT_MOTOR;
        }
    }

    public class PCM {

    }

    public class DIO {

    }

    public class PWM {
        public LedMap ledMap = new LedMap();
      
        public class LedMap{
            public int LED_CONTROLLER;
        }
    }
}
