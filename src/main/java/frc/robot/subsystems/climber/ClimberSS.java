package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.PWMSparkMax;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class ClimberSS extends OverridableSubsystem {
    private PWMSparkMax motor;
    private RobotConstants.ClimberConstants constants;

    public ClimberSS(RobotConstants.ClimberConstants constants) {
        this.constants = constants;
        motor = constants.PWM_MAP.MOTOR;
        motor.setInverted(constants.IS_INVERTED);
    }

  /**
   *
   * @param power to be set to the motor (between -1 and 1)
   */
    public void overriddenMove(double power) {
        motor.set(power);
    }
}
