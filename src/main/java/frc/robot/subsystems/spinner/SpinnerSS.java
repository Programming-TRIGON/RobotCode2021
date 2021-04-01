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
//    private final TrigonDoubleSolenoid solenoid;
//    private final ColorSensorV3 colorSensor;

    /*
    <NOTE>
    The code for the "Colour Spinney Thing" - Krombein,N(2021) is not relevant for the current challenges therefore have been commented out.
    Will be used when coding for the competition
     */
    public SpinnerSS(SpinnerConstants constants) {
        this.constants = constants;
        motor = constants.CAN_MAP.MOTOR;
//        solenoid = constants.PCM_MAP.RIGHT_SOLENOID;
//        colorSensor = constants.I2C_MAP.COLOR_SENSOR;
    }

    /**
     * @param power to bve set to the motor
     */
    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }

//    /**
//     * Sets the state of the solenoid with a boolean
//     *
//     * @param state to be set to the solenoid (true=roulette false=mixer)
//     */
//    public void setSolenoidState(boolean state) {
//        solenoid.setSolenoid(state);
//    }
//
//    /**
//     * Gets the state of the solenoid as a boolean
//     *
//     * @return the state of the solenoid (true=roulette false=mixer)
//     */
//    @Log(name = "Spinner mode")
//    public boolean getSolenoidState() {
//        return solenoid.isForward();
//    }

    @Log(name = "Stator current")
    public double getStatorCurrent() {
        return motor.getStatorCurrent();
    }

    @Log(name = "Is stalled")
    public boolean isStalled() {
        return motor.getStatorCurrent() > constants.STALL_CURRENT_LIMIT;
    }

//    /**
//     * @return the raw color inputs from the color sensor
//     */
//    @Log(name = "Raw color value")
//    public RawColor getRawColor() {
//        return colorSensor.getRawColor();
//    }
//
//    /**
//     * @return the color that the color sensor sees
//     */
//    @Log(name = "Color")
//    public Color getColor() {
//        return colorSensor.getColor();
//    }
//
//    /**
//     * @return distance from color sensor from the target
//     * as the target is closer the values it larger (between 0 to 2047)
//     */
//    public int getProximity() {
//        return colorSensor.getProximity();
//    }

    @Override
    public String configureLogName() {
        return "Spinner";
    }
}
