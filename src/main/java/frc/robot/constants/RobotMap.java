package frc.robot.constants;

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

    }

    public class PCM {

    }

    public class DIO {

    }

    public class PWM {
        public LED led = new LED();
        public class LED{
            public int LED_CONTROLLER;
        }
    }
}
