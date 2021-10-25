package frc.robot.subsystems.climber;

import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class WinchSS extends OverridableSubsystem {
    private final ClimberConstants constants;
    private final TrigonTalonSRX motor;

    public WinchSS(ClimberConstants constants) {
        this.constants = constants;
        motor = constants.CAN_MAP.WINCH_MOTOR;
    }

    public void overriddenMove(double power) {
        motor.set(power);
    }

    public double getStatorCurrent() {
        return motor.getStatorCurrent();
    }

    public boolean isStalled() {
        return  Math.abs(motor.getStatorCurrent()) > constants.WINCH_STALL_LIMIT;
    }
}
