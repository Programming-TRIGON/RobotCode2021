package frc.robot.subsystems.intake;

import frc.robot.components.TrigonTalonSRX;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.constants.RobotConstants.IntakeConstants;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeSS extends OverridableSubsystem {
    private final IntakeConstants constants;
    private final TrigonTalonSRX motor;

    public IntakeSS(IntakeConstants constants) {
        this.constants = constants;
        motor = constants.CAN_MAP.MOTOR;
    }

    public void overriddenMove(double power) {
        motor.set(power);
    }

    @Log(name = "Intake/Stator Current")
    public double getStatorCurrent() {
        return motor.getStatorCurrent();
    }

    @Log(name = "Intake/Is Stalled")
    public boolean isStalled() {
        return motor.getStatorCurrent() > constants.STALL_CURRENT_LIMIT;
    }
}
