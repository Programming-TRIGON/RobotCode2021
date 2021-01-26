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

    rightMotor = constants.CAN_MAP.RIGHT_MOTOR;
    leftMotor = constants.CAN_MAP.LEFT_MOTOR;
    masterMotor = rightMotor;

    rightMotor.follow(masterMotor);
    leftMotor.follow(masterMotor);
  }

  /**
   * 
   * @param power to be set to the motors (between -1 and 1)
   */
  public void move(double power) {
    masterMotor.set(power);
  }

  /**
   * 
   * @return the velocity of the motors in ticks/centiseconds
   */
  public double getVelocity() {
    return masterMotor.getSelectedSensorVelocity();
  }

  /**
   * 
   * @return an array of the current encoder position
   */
  public double[] getValues() {
    return new double[] { masterMotor.getSelectedSensorPosition() };
  }
}
