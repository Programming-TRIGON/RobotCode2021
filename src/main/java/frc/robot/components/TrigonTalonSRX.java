package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.PIDCoefs;

/**
 * This class creates a new instance of WPI_TalonSRX and configures values based
 * on a config preset
 */
public class TrigonTalonSRX extends WPI_TalonSRX {
    private PIDCoefs pidCoefs;

    /**
     * constructs a new motor controller
     * 
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     * @param pidCoefs    coefficients for CTRE PID
     */
    public TrigonTalonSRX(int id, MotorConfig motorConfig, PIDCoefs pidCoefs) {
        super(id);
        this.pidCoefs = pidCoefs;
        configOpenloopRamp(motorConfig.getRampRate());
        configClosedloopRamp(motorConfig.getRampRate());
        setInverted(motorConfig.isInverted());
        setSensorPhase(motorConfig.isSensorInverted());
        setNeutralMode(motorConfig.getNeutralMode());
        configVoltageCompSaturation(motorConfig.getVoltageCompSaturation());
        enableVoltageCompensation(motorConfig.getVoltageCompSaturation() > 0);
        configSupplyCurrentLimit(motorConfig.getSupplyCurrentLimitConfiguration());
        configurePID();
    }

    /**
     * constructs a new motor controller with a default PID configurations
     * 
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     */
    public TrigonTalonSRX(int id, MotorConfig motorConfig) {
        this(id, motorConfig, new PIDCoefs(0, 0, 0, 0, 0, 0));
    }

    /**
     * constructs a new motor controller with a default motor configurations
     * 
     * @param id       device ID of motor controller
     * @param pidCoefs coefficients for CTRE PID
     */
    public TrigonTalonSRX(int id, PIDCoefs pidCoefs) {
        this(id, new MotorConfig(), pidCoefs);
    }

    /**
     * constructs a new motor controller with a default configurations
     * 
     * @param id device ID of motor controller
     */
    public TrigonTalonSRX(int id) {
        this(id, new MotorConfig(), new PIDCoefs(0, 0, 0, 0, 0, 0));
    }

    /**
     * configures the PID coefficients
     */
    public void configurePID() {
        config_kP(0, pidCoefs.getKP());
        config_kI(0, pidCoefs.getKI());
        config_kD(0, pidCoefs.getKD());
        config_kF(0, pidCoefs.getKF());
    }

    /**
     * sets new PID coefficients
     * 
     * @param pidCoefs the coefficients fot the PID
     */
    public void configurePID(PIDCoefs pidCoefs) {
        this.pidCoefs = pidCoefs;
        configurePID();
    }

    /**
     * if tuning the PID then call periodically
     * 
     * @param name name of the key
     */
    public void tunePID(String name) {
        SmartDashboard.putData(name + "/PIDCoefs", pidCoefs);
        configurePID();
    }
}