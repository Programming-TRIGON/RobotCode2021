package frc.robot.components;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.PIDFCoefs;

/**
 * This class creates a new instance of WPI_TalonFX and configures values based
 * on a config preset
 */
public class TrigonTalonFX extends WPI_TalonFX {
    private final MotorConfig motorConfig;
    private PIDFCoefs pidCoefs;

    /**
     * constructs a new motor controller
     *
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     * @param pidCoefs    coefficients for CTRE PID
     */
    public TrigonTalonFX(int id, MotorConfig motorConfig, PIDFCoefs pidCoefs) {
        super(id);
        this.motorConfig = motorConfig;
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
    public TrigonTalonFX(int id, MotorConfig motorConfig) {
        this(id, motorConfig, new PIDFCoefs(0, 0, 0, 0, 0, 0));
    }

    /**
     * constructs a new motor controller with a default motor configurations
     *
     * @param id       device ID of motor controller
     * @param pidCoefs coefficients for CTRE PID
     */
    public TrigonTalonFX(int id, PIDFCoefs pidCoefs) {
        this(id, new MotorConfig(), pidCoefs);
    }

    /**
     * constructs a new motor controller with a default configurations
     *
     * @param id device ID of motor controller
     */
    public TrigonTalonFX(int id) {
        this(id, new MotorConfig(), new PIDFCoefs(0, 0, 0, 0, 0, 0));
    }

    public NeutralMode getNeutralMode() {
        return motorConfig.getNeutralMode();
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
     * Sets new PID coefficients
     *
     * @param pidCoefs the coefficients fot the PID
     */
    public void configurePID(PIDFCoefs pidCoefs) {
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
