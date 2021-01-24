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
    motor = constants.CAN_MAP.motor;
  }

  public void setMotor(double power) {
    motor.set(power);
  }

  public double getVelocity() {
    return motor.getSelectedSensorVelocity();
  }

  public double[] getValues() {
    return new double[] { motor.getSelectedSensorPosition() };
  }

  public void overriddenMove(double power) {
    motor.set(power);
  }

}
