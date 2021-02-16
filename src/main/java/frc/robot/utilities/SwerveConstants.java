package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.components.MotorConfig;
import frc.robot.components.TrigonTalonFX;

public class SwerveConstants {

    public TrigonTalonFX speedMotor;
    public TrigonTalonFX angleMotor;
    public double diameter;
    public double offset;
    public PIDCoefs angleCoefs;
    public PIDCoefs speedCoefs;
    public boolean tuning;

    public SwerveConstants(TrigonTalonFX speedMotor, TrigonTalonFX angleMotor, double diameter, double offset, PIDCoefs angleCoefs, PIDCoefs speedCoefs, boolean tuning) {
        this.speedMotor = speedMotor;
        this.angleMotor = angleMotor;
        this.diameter = diameter;
        this.offset = offset;
        this.angleCoefs = angleCoefs;
        this.speedCoefs = speedCoefs;
        this.tuning = tuning;
    }

    public static class StaticSwerveConstants {
        public static final FeedbackDevice ABSOLUTE_DEVICE = FeedbackDevice.CTRE_MagEncoder_Absolute;
        public static final FeedbackDevice RELATIVE_DEVICE = FeedbackDevice.CTRE_MagEncoder_Relative;
        public static int SPEED_MOTOR_TICKS_PER_REVOLUTION;
        public static int ANGLE_TICKS_PER_REVOLUTION;
        public static double SPEED_GEAR_RATION;
        public static MotorConfig ANGLE_DEFAULT_CONFIG;
        public static MotorConfig SPEED_DEFAULT_CONFIG;
    }
}
