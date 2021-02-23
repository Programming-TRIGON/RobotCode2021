package frc.robot.subsystems.spinner;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.components.TrigonDoubleSolenoid;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.SpinnerConstants;
import frc.robot.subsystems.OverridableSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class SpinnerSS extends OverridableSubsystem implements Loggable {
    private final SpinnerConstants constants;
    private final TrigonTalonSRX motor;
    private final TrigonDoubleSolenoid solenoid;
    private final ColorSensorV3 colorSensor;

    public SpinnerSS(SpinnerConstants constants) {
        this.constants = constants;
        motor = constants.CAN_MAP.MOTOR;
        solenoid = constants.PCM_MAP.SOLENOID;
        colorSensor = constants.I2C_MAP.COLOR_SENSOR;
    }

    /**
     * @param power to bve set to the motor
     */
    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }

    /**
     * Sets the state of the solenoid with a boolean
     *
     * @param state to be set to the solenoid (true=forward false=reverse)
     */
    public void setSolenoidState(boolean state) {
        solenoid.setSolenoid(state);
    }

    /**
     * Gets the state of the solenoid as a boolean
     *
     * @return the state of the solenoid (true=forward false=reverse)
     */
    @Log(name = "Is open")
    public boolean getSolenoidState() {
        return solenoid.isForward();
    }

    /**
     * @return the raw color inputs from the color sensor
     */
    public RawColor getRawColor() {
        return colorSensor.getRawColor();
    }

    /**
     * @return the color that the color sensor sees
     */
    public Color getColor() {
        return colorSensor.getColor();
    }
}

