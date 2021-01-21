package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonTalonFX;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.TestableSubsystem;

public class ShooterSS extends SubsystemBase implements TestableSubsystem {
  private RobotConstants.ShooterConstants constants;

  private TrigonTalonFX rightMotor;
  private TrigonTalonFX leftMotor;
  private TrigonTalonFX masterMotor;

  public ShooterSS(RobotConstants.ShooterConstants constants) {
    this.constants = constants;

    rightMotor = constants.canShooterMap.RIGHT_MOTOR;
    leftMotor = constants.canShooterMap.LEFT_MOTOR;
    masterMotor = rightMotor;

    rightMotor.follow(masterMotor);
    leftMotor.follow(rightMotor);
  }

  public void setMotors(double power) {
    masterMotor.set(power);
  }

  public double getVelocityInRPM() {
    return masterMotor.getSelectedSensorVelocity() * constants.CENTISECONDS_IN_MINUTE * constants.TICKS_PER_REVOLUTION;
  }

  public double[] getValues() {
    return new double[] { masterMotor.getSelectedSensorPosition() };
  }

  public void move(double power) {
    masterMotor.set(power);
  }
}
