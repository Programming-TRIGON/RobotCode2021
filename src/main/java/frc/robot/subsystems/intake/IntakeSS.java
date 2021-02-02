package frc.robot.subsystems.intake;

import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class IntakeSS extends OverridableSubsystem {
    private RobotConstants.IntakeConstants constants;
    private TrigonTalonSRX motor;

    public IntakeSS(RobotConstants.IntakeConstants constants) {
        this.constants = constants;
        motor = constants.CAN_MAP.MOTOR;
    }

    public void overriddenMove(double power) {
        motor.set(power);
    }

    public boolean isStalled() {
        return motor.getStatorCurrent() > constants.ON_STALL_CURRENT_LIMIT;
    }
}