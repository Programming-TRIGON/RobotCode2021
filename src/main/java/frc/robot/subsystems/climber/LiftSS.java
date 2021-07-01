package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.PWMSparkMax;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class LiftSS extends OverridableSubsystem {
    private final ClimberConstants constants;
    private final TrigonTalonSRX motor;

    public LiftSS(ClimberConstants constants) {
        this.constants = constants;
        motor = constants.CAN_MAP.LIFT_MOTOR;
    }

    /**
     *
     * @param power to be set to the motor (between -1 and 1)
     */
    public void overriddenMove(double power) {
        motor.set(power);
    }
}
