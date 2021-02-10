package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.components.TrigonTalonFX;

public class SwerveConstants {

    public TrigonTalonFX speedMotor, angleMotor;
    public double diameter, offset;
    public PIDCoefs angleCoefs, speedCoefs;


    public SwerveConstants(TrigonTalonFX speedMotor, TrigonTalonFX angleMotor, double diameter, double offset, PIDCoefs angleCoefs, PIDCoefs speedCoefs) {
        this.speedMotor = speedMotor;
        this.angleMotor = angleMotor;
        this.diameter = diameter;
        this.offset = offset;
        this.angleCoefs = angleCoefs;
        this.speedCoefs = speedCoefs;
    }

    public static class StaticSwerveConstants {
        public static final FeedbackDevice
                ABSOLUTE_DEVICE = FeedbackDevice.CTRE_MagEncoder_Absolute,
                RELATIVE_DEVICE = FeedbackDevice.CTRE_MagEncoder_Relative;
        public static int
                SPEED_TICKS_PER_REVOLUTION,
                ANGLE_TICKS_PER_REVOLUTION;
    }
}
