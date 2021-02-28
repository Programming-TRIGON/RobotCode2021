package frc.robot.subsystems.intakeOpener;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utilities.DriverStationLogger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeOpenerSS extends OverridableSubsystem implements Loggable {
    private final TrigonTalonSRX motor;
    private final DigitalInput closedSwitch;
    private final DigitalInput openSwitch;
    private final IntakeOpenerConstants constants;

    public IntakeOpenerSS(IntakeOpenerConstants constants) {
        motor = constants.CAN_MAP.MOTOR;
        closedSwitch = constants.DIO_MAP.CLOSED_SWITCH;
        openSwitch = constants.DIO_MAP.OPEN_SWITCH;
        this.constants = constants;
    }

    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }

    /**
     * @return is the intake closed
     */
    @Log(name = "Intake closed")
    public boolean isClosed() {
        return closedSwitch.get();
    }

    /**
     * @return is the intake open
     */
    @Log(name = "Intake open")
    public boolean isOpen() {
        return openSwitch.get();
    }

    /**
     * Sets the motor power only if the power given is appropriate
     *
     * @param power to be set to the motor
     */
    public void moveWithSafety(double power) {
        if (isClosed() && power >= 0 || isOpen() && power < 0)
            motor.set(power);
        else
            DriverStationLogger.logToDS("moveWithSafety: invalid power given");
    }

    @Override
    public String configureLogName() {
        return constants.LOGGABLE_NAME;
    }
}
