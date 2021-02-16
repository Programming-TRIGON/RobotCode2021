package frc.robot.subsystems.intakeOpener;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;
import frc.robot.subsystems.OverridableSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeOpenerSS extends OverridableSubsystem implements Loggable {
    private final TrigonTalonSRX motor;
    private final DigitalInput closedInput;
    private final DigitalInput openInput;
    private final IntakeOpenerConstants constants;

    public IntakeOpenerSS(IntakeOpenerConstants constants) {
        this.motor = constants.CAN_MAP.MOTOR;
        this.closedInput = constants.DIO_MAP.CLOSED_INPUT;
        this.openInput = constants.DIO_MAP.OPEN_INPUT;
        this.constants = constants;
    }

    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }

    /**
     * @return is the intake closed
     */
    @Log(name = "Is Closed")
    public boolean isClosed() {
        return closedInput.get();
    }

    /**
     * @return is the intake open
     */
    @Log(name = "Is Open")
    public boolean isOpen() {
        return openInput.get();
    }

    /**
     * @return is the intake is not moving (either open or closed)
     */
    public boolean isStill() {
        return isClosed() || isOpen();
    }

    /**
     * Sets the motor power only if the power given is appropriate
     *
     * @param power to be set to the motor
     */
    public void moveWithSafety(double power) {
        if (isStill()) {
            if (isClosed() && power >= 0)
                motor.set(power);
            else if (isOpen() && power < 0)
                motor.set(power);
            else
                System.out.println("moveWithSafety: invalid power given");
        }
        else
            System.out.println("moveWithSafety: intake was moving");
    }
}

