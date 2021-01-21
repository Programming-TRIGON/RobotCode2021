package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonTalonFX;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.TestableSubsystem;

public class ShooterSS extends SubsystemBase implements TestableSubsystem {
  private RobotConstants.ShooterConstants constants;

  private TrigonTalonFX rightMotor;
  private TrigonTalonFX leftMotor;

  public ShooterSS(RobotConstants.ShooterConstants constants) {
    this.constants = constants;

    rightMotor = constants.canShooterMap.RIGHT_MOTOR;
    leftMotor = constants.canShooterMap.LEFT_MOTOR;

    leftMotor.follow(rightMotor);
  }

  public void setMotors(double power) {
    rightMotor.set(power);
  }

  public double getVelocityInRPM() {
    return rightMotor.getSelectedSensorVelocity() * constants.CENTISECONDS_IN_MINUTE * constants.TICKS_PER_REVOLUTION;
  }

  public double[] getValues() {
    return new double[] { rightMotor.getSelectedSensorPosition() };
  }

  public void move(double power) {
    rightMotor.set(power);
  }
}
