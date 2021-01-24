package frc.robot.subsystems.intake;

import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class IntakeSS extends OverridableSubsystem {
  private TrigonTalonSRX motor;

  public IntakeSS(RobotConstants.IntakeConstants intakeConstants) {
    motor = intakeConstants.CAN_MAP.MOTOR;
  }
  
  public void overriddenMove(double power) {
    motor.set(power);
  }
}