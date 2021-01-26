package frc.robot.subsystems.trigger;

import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import io.github.oblarg.oblog.Loggable;

public class TriggerSS extends OverridableSubsystem implements TestableSubsystem, Loggable {
  private RobotConstants.TriggerConstants constants;
  private TrigonTalonSRX motor;

  public TriggerSS(RobotConstants.TriggerConstants constants) {
    this.constants = constants;
    motor = constants.CAN_MAP.MOTOR;
  }

  /**
   * 
   * @param power to be set to the motor (between -1 and 1)
   */
  public void overriddenMove(double power) {
    motor.set(power);
  }

  /**
   * 
   * @return the velocity of the motor in ticks/centisecond
   */
  public double getVelocity() {
    return motor.getSelectedSensorVelocity();
  }

  /**
   * 
   * @return the position of the motor in an array
   */
  public double[] getValues() {
    return new double[] { motor.getSelectedSensorPosition() };
  }

}