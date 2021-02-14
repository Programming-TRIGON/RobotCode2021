package frc.robot.subsystems.intakeOpener;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class IntakeOpenerSS extends OverridableSubsystem {
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

    public boolean areSensorsPressed() {
        return rightInput.get() || leftInput.get();
    }
}

