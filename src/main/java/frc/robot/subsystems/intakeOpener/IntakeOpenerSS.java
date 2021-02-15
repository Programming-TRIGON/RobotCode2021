package frc.robot.subsystems.intakeOpener;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;
import frc.robot.subsystems.OverridableSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeOpenerSS extends OverridableSubsystem implements Loggable {
    private final TrigonTalonSRX motor;
    private final DigitalInput rightInput;
    private final DigitalInput leftInput;
    private final IntakeOpenerConstants constants;

    public IntakeOpenerSS(IntakeOpenerConstants constants) {
        this.motor = constants.CAN_MAP.MOTOR;
        this.rightInput = constants.DIO_MAP.RIGHT_INPUT;
        this.leftInput = constants.DIO_MAP.LEFT_INPUT;
        this.constants = constants;
    }

    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }

    @Log(name = "IntakeOpener/Is Right Sensor Pressed")
    public boolean isRightSensorPressed() {
        return rightInput.get();
    }

    @Log(name = "IntakeOpener/Is Left Sensor Pressed")
    public boolean isLeftSensorPressed() {
        return leftInput.get();
    }

    @Log(name = "IntakeOpener/Is Either Sensor Pressed")
    public boolean isEitherSensorPressed() {
        return rightInput.get() || leftInput.get();
    }

    @Log(name = "IntakeOpener/Are Both Sensors Pressed")
    public boolean areBothSensorsPressed() {
        return rightInput.get() && leftInput.get();
    }

    public void moveWithSafety(double power) {
        if (!isEitherSensorPressed() && power >= 0)
            motor.set(power);
        else if (isEitherSensorPressed() && power < 0)
            motor.set(power);
        else {
            // Place holder TODO: put what needs to be put
            return;
        }
    }
}

