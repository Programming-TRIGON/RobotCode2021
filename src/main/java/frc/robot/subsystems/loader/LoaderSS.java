package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class LoaderSS extends OverridableSubsystem implements TestableSubsystem, Loggable {
    private final LoaderConstants constants;
    private final TrigonTalonSRX motor;

    public LoaderSS(LoaderConstants constants) {
        this.constants = constants;
        motor = constants.CAN_MAP.MOTOR;
    }

    /**
     * Sets the motor's velocity using PID
     *
     * @param desiredVelocity desired velocity of the motor in ticks/second
     */
    public void setDesiredVelocity(double desiredVelocity) {
        if (!overridden)
            // velocity is in ticks/seconds and is divided by 10 to convert to ticks/100ms
            motor.set(ControlMode.Velocity, desiredVelocity / 10);
    }


    /**
     * @param power to be set to the motor (between -1 and 1)
     */
    public void overriddenMove(double power) {
        motor.set(power);
    }

    /**
     * @return the velocity of the motor in ticks/seconds
     */
    @Log(name = "Trigger/Motor Velocity")
    public double getVelocity() {
        // motor.getSelectedSensorVelocity() returns in ticks/100ms then is multiplied by 10 to convert to ticks/second
        return motor.getSelectedSensorVelocity() * 10;
    }

    /**
     * @return the position of the motor in an array
     */
    public double[] getValues() {
        return new double[]{motor.getSelectedSensorPosition()};
    }
}
