package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * This class creates a new instance of WPI_TalonFX and configures values based
 * on a config preset
 */
public class TrigonTalonFX extends WPI_TalonFX {

    /**
     * constructs a new motor controller
     * 
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     */
    public TrigonTalonFX(int id, MotorConfig motorConfig) {
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
    public TrigonTalonFX(int id) {
        this(id, new MotorConfig());
    }
}