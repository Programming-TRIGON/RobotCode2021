package frc.robot.components;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/**
 * This class stores the configuration of a motor
 */
public class MotorConfig {

    private double rampRate;
    private boolean isInverted;
    private boolean isSensorInverted;
    private NeutralMode neutralMode;
    private double voltageCompSaturation;
    private SupplyCurrentLimitConfiguration currentLimitConfig;

    /**
     * This constructor uses default values for all settings disables rampRate,
     * voltage compensation saturation, and supplyCurrentLimitConfiguration, sets
     * neutral mode to coast, and sets all inverts to false.
     */
    public MotorConfig() {
        this(0, NeutralMode.Coast, 0);
    }

    /**
     * This constructor copies settings from another motorConfig and sets sensor
     * invert to false. Use this constructor when you have multiple motors that need
     * the same settings but different inverts and the motor does not have a sensor
     * connected to it.
     * 
     * @param motorConfig motorConfig to copy voltage compensation saturation,
     *                    supply current limit configuration, and neutral mode
     * @param isInverted  Inverts the hbridge output of the motor controller.
     */
    public MotorConfig(MotorConfig motorConfig, boolean isInverted) {
        this(motorConfig.getRampRate(), isInverted, false, motorConfig.getNeutralMode(),
                motorConfig.getVoltageCompSaturation(), motorConfig.getSupplyCurrentLimitConfiguration());
    }

    /**
     * This constructor copies settings from another motorConfig. Use this
     * constructor when you have multiple motors that need the same settings but
     * different inverts.
     * 
     * @param motorConfig      motorConfig to copy voltage compensation saturation,
     *                         supply current limit configuration, and neutral mode
     * @param isInverted       Inverts the hbridge output of the motor controller.
     * @param isSensorInverted Sets the phase of the sensor. Use when controller
     *                         forward/reverse output doesn't correlate to
     *                         appropriate forward/reverse reading of sensor. Pick a
     *                         value so that positive PercentOutput yields a
     *                         positive change in sensor.
     */
    public MotorConfig(MotorConfig motorConfig, boolean isInverted, boolean isSensorInverted) {
        this(motorConfig.getRampRate(), isInverted, isSensorInverted, motorConfig.getNeutralMode(),
                motorConfig.getVoltageCompSaturation(), motorConfig.getSupplyCurrentLimitConfiguration());
    }

    /**
     * 
     * @param rampRate              Minimum desired time to go from neutral to full
     *                              throttle. A value of '0' will disable the ramp.
     * @param neutralMode           The desired mode of operation when the
     *                              Controller output throttle is neutral (ie
     *                              brake/coast)
     * @param voltageCompSaturation This is the max voltage to apply to the hbridge
     *                              when voltage compensation is enabled. For
     *                              example, if 10 (volts) is specified and a motor
     *                              controller is commanded to 0.5 (PercentOutput,
     *                              closed-loop, etc) then the motor controller will
     *                              attempt to apply a duty-cycle to produce 5V. A
     *                              value of 0 disables this feature.
     */
    public MotorConfig(double rampRate, NeutralMode neutralMode, double voltageCompSaturation) {
        this(rampRate, false, false, neutralMode, voltageCompSaturation);
    }

    /**
     * 
     * @param rampRate              Minimum desired time to go from neutral to full
     *                              throttle. A value of '0' will disable the ramp.
     * @param isInverted            Inverts the hbridge output of the motor
     *                              controller.
     * @param isSensorInverted      Sets the phase of the sensor. Use when
     *                              controller forward/reverse output doesn't
     *                              correlate to appropriate forward/reverse reading
     *                              of sensor. Pick a value so that positive
     *                              PercentOutput yields a positive change in
     *                              sensor.
     * @param neutralMode           The desired mode of operation when the
     *                              Controller output throttle is neutral (ie
     *                              brake/coast)
     * @param voltageCompSaturation This is the max voltage to apply to the hbridge
     *                              when voltage compensation is enabled. For
     *                              example, if 10 (volts) is specified and a motor
     *                              controller is commanded to 0.5 (PercentOutput,
     *                              closed-loop, etc) then the motor controller will
     *                              attempt to apply a duty-cycle to produce 5V. A
     *                              value of 0 disables this feature.
     */
    public MotorConfig(double rampRate, boolean isInverted, boolean isSensorInverted, NeutralMode neutralMode,
            double voltageCompSaturation) {
        this(rampRate, isInverted, isSensorInverted, neutralMode, voltageCompSaturation,
                new SupplyCurrentLimitConfiguration(false, 0, 0, 0));
    }

    /**
     * 
     * @param rampRate                        Minimum desired time to go from
     *                                        neutral to full throttle. A value of
     *                                        '0' will disable the ramp.
     * @param neutralMode                     The desired mode of operation when the
     *                                        Controller output throttle is neutral
     *                                        (ie brake/coast)
     * @param voltageCompSaturation           This is the max voltage to apply to
     *                                        the hbridge when voltage compensation
     *                                        is enabled. For example, if 10 (volts)
     *                                        is specified and a motor controller is
     *                                        commanded to 0.5 (PercentOutput,
     *                                        closed-loop, etc) then the motor
     *                                        controller will attempt to apply a
     *                                        duty-cycle to produce 5V. A value of 0
     *                                        disables this feature.
     * @param supplyCurrentLimitConfiguration Supply-side current limiting. This is
     *                                        typically used to prevent breakers
     *                                        from tripping.
     */
    public MotorConfig(double rampRate, NeutralMode neutralMode, double voltageCompSaturation,
            SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
        this(rampRate, false, false, neutralMode, voltageCompSaturation, supplyCurrentLimitConfiguration);
    }

    /**
     * 
     * @param rampRate                        Minimum desired time to go from
     *                                        neutral to full throttle. A value of
     *                                        '0' will disable the ramp.
     * @param isInverted                      Inverts the hbridge output of the
     *                                        motor controller.
     * @param isSensorInverted                Sets the phase of the sensor. Use when
     *                                        controller forward/reverse output
     *                                        doesn't correlate to appropriate
     *                                        forward/reverse reading of sensor.
     *                                        Pick a value so that positive
     *                                        PercentOutput yields a positive change
     *                                        in sensor.
     * @param neutralMode                     The desired mode of operation when the
     *                                        Controller output throttle is neutral
     *                                        (ie brake/coast)
     * @param voltageCompSaturation           This is the max voltage to apply to
     *                                        the hbridge when voltage compensation
     *                                        is enabled. For example, if 10 (volts)
     *                                        is specified and a motor controller is
     *                                        commanded to 0.5 (PercentOutput,
     *                                        closed-loop, etc) then the motor
     *                                        controller will attempt to apply a
     *                                        duty-cycle to produce 5V. A value of
     *                                        '0' disables this feature.
     * @param supplyCurrentLimitConfiguration Supply-side current limiting. This is
     *                                        typically used to prevent breakers
     *                                        from tripping.
     */
    public MotorConfig(double rampRate, boolean isInverted, boolean isSensorInverted, NeutralMode neutralMode,
            double voltageCompSaturation, SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
        this.rampRate = rampRate;
        this.isInverted = isInverted;
        this.isSensorInverted = isSensorInverted;
        this.voltageCompSaturation = voltageCompSaturation;
        this.currentLimitConfig = supplyCurrentLimitConfiguration;
    }

    public double getRampRate() {
        return this.rampRate;
    }

    public boolean isInverted() {
        return this.isInverted;
    }

    public boolean isSensorInverted() {
        return this.isSensorInverted;
    }

    public NeutralMode getNeutralMode() {
        return this.neutralMode;
    }

    public SupplyCurrentLimitConfiguration getSupplyCurrentLimitConfiguration() {
        return this.currentLimitConfig;
    }

    public double getVoltageCompSaturation() {
        return this.voltageCompSaturation;
    }
}