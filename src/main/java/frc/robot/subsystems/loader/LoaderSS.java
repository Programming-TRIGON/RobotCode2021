package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class LoaderSS extends OverridableSubsystem implements TestableSubsystem, Loggable {
    private RobotConstants.LoaderConstants constants;
    private TrigonTalonSRX motor;

    public LoaderSS(RobotConstants.LoaderConstants constants) {
        this.constants = constants;
        motor = constants.CAN_MAP.MOTOR;
    }

    /**
     * Sets the motor's velocity using PID
     *
     * @param velocity desired velocity of the motor
     */
    public void setVelocity(double velocity) {
        if (!overridden)
            motor.set(ControlMode.Velocity, velocity);
    }

    /**
     * @param power to be set to the motor (between -1 and 1)
     */
    public void overriddenMove(double power) {
        motor.set(power);
    }

    /**
     * @return the velocity of the motor in ticks/centisecond
     */
    @Log(name = "Trigger/Motor Velocity")
    public double getVelocity() {
        return motor.getSelectedSensorVelocity();
    }

    /**
     * @return the position of the motor in an array
     */
    public double[] getValues() {
        return new double[] { motor.getSelectedSensorPosition() };
    }
}
