package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.PWMSparkMax;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class ClimberSS extends OverridableSubsystem {
    private PWMSparkMax sparkMax;

    public ClimberSS(RobotConstants.ClimberConstants climberConstants) {
        sparkMax = climberConstants.PWM_MAP.MOTOR;
        sparkMax.setInverted(climberConstants.IS_INVERTED);
    }

  /**
   *
   * @param power to be set to the motor (between -1 and 1)
   */
    public void overriddenMove(double power) {
        sparkMax.set(power);
    }
}
