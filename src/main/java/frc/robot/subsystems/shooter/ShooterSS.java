package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonTalonFX;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.TestableSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterSS extends SubsystemBase implements TestableSubsystem, Loggable {
    private final ShooterConstants constants;
    private final TrigonTalonFX masterMotor;

    public ShooterSS(ShooterConstants constants) {
        this.constants = constants;
        masterMotor = constants.CAN_MAP.RIGHT_MOTOR;
        constants.CAN_MAP.RIGHT_MOTOR.follow(masterMotor);
        constants.CAN_MAP.LEFT_MOTOR.follow(masterMotor);
    }

    /**
     * @param voltage to be set to the motors
     */
    public void move(double voltage) {
        masterMotor.setVoltage(voltage);
    }

    /**
     * @return current power of the motor (between -1 and 1)
     */
    public double getPower() {
        return masterMotor.get();
    }

    /**
     * @return the velocity of the motors in ticks/100 milliseconds
     */
    @Log(name = "Shooter/Velocity")
    public double getVelocity() {
        return masterMotor.getSelectedSensorVelocity() * 600 / 2048;
    }

    public void setRampRate(double rampRate) {
        constants.CAN_MAP.RIGHT_MOTOR.configClosedloopRamp(rampRate);
        constants.CAN_MAP.LEFT_MOTOR.configClosedloopRamp(rampRate);
        constants.CAN_MAP.RIGHT_MOTOR.configOpenloopRamp(rampRate);
        constants.CAN_MAP.LEFT_MOTOR.configOpenloopRamp(rampRate);
    }

    /**
     * @return an array of the current encoder position
     */
    public double[] getValues() {
        return new double[] { masterMotor.getSelectedSensorPosition() };
    }
}
