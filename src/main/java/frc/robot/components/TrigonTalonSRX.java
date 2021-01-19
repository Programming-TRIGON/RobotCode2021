package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * This class creates a new instance of WPI_TalonSRX and configures values based
 * on a config preset
 */
public class TrigonTalonSRX extends WPI_TalonSRX {

    /**
     * constructs a new motor controller
     * 
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     */
    public TrigonTalonSRX(int id, MotorConfig motorConfig) {
        super(id);
        configOpenloopRamp(motorConfig.getRampRate());
        configClosedloopRamp(motorConfig.getRampRate());
        setInverted(motorConfig.isInverted());
        setSensorPhase(motorConfig.isInverted());
        setNeutralMode(motorConfig.getNeutralMode());
        configVoltageCompSaturation(motorConfig.getVoltageCompSaturation());
        enableVoltageCompensation(motorConfig.getVoltageCompSaturation() > 0);
        configSupplyCurrentLimit(motorConfig.getSupplyCurrentLimitConfiguration());
    }

    /**
     * constructs a new motor controller with a default configuration
     * 
     * @param id device ID of motor controller
     */
    public TrigonTalonSRX(int id) {
        this(id, new MotorConfig());
    }
}