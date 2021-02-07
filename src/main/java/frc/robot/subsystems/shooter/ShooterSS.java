package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonTalonFX;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.TestableSubsystem;

public class ShooterSS extends SubsystemBase implements TestableSubsystem {
    private final RobotConstants.ShooterConstants constants;
    private final TrigonTalonFX masterMotor;

    public ShooterSS(RobotConstants.ShooterConstants constants) {
        this.constants = constants;
        masterMotor = constants.CAN_MAP.RIGHT_MOTOR;
        constants.CAN_MAP.RIGHT_MOTOR.follow(masterMotor);
        constants.CAN_MAP.LEFT_MOTOR.follow(masterMotor);
    }

    /**
     * @param power to be set to the motors (between -1 and 1)
     */
    public void move(double power) {
        masterMotor.set(power);
    }

    /**
     * Sets the desired velocity of the motors
     *
     * @param velocity to be set to the motors
     */
    public void setVelocity(double velocity) {
        masterMotor.set(ControlMode.Velocity, velocity);
    }

    /**
     * @return the velocity of the motors in ticks/seconds
     */
    public double getVelocity() {
        return masterMotor.getSelectedSensorVelocity() * 10;
    }

    /**
     * @return an array of the current encoder position
     */
    public double[] getValues() {
        return new double[]{masterMotor.getSelectedSensorPosition()};
    }
}
