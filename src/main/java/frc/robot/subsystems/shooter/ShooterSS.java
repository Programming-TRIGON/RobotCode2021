package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonTalonFX;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotMap.CAN;

public class ShooterSS extends SubsystemBase {
  private RobotConstants.ShooterConstants constants;

  private TrigonTalonFX rightMotor;
  private TrigonTalonFX leftMotor;

  public ShooterSS(RobotConstants.ShooterConstants constants, CAN.ShooterMap robotMap) {
    this.constants = constants;

    rightMotor = new TrigonTalonFX(robotMap.RIGHT_MOTOR_ID, constants.RIGHT_MOTOR_CONFIG);
    leftMotor = new TrigonTalonFX(robotMap.LEFT_MOTOR_ID, constants.LEFT_MOTOR_CONFIG);

    leftMotor.follow(rightMotor);
  }

  public void setMotors(double power) {
    rightMotor.set(power);
  }

  public double getVelocityInRPM() {
    return rightMotor.getSelectedSensorVelocity() * constants.MILISECONDS_IN_MINUTE * constants.TICKS_PER_REVOLUTION;
  }

  public int getRightEncoderPosition() {
    return rightMotor.getSelectedSensorPosition();
  }

  public int getLeftEncoderPosition() {
    return leftMotor.getSelectedSensorPosition();
  }
}
