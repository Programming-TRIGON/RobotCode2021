package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.subsystems.KfCalibratableSubsystem;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class LoaderSS extends OverridableSubsystem implements TestableSubsystem, Loggable, KfCalibratableSubsystem {
    private final LoaderConstants constants;
    private final TrigonTalonSRX motor;

    public LoaderSS(LoaderConstants constants) {
        this.constants = constants;
        motor = constants.CAN_MAP.MOTOR;
    }

    /**
     * Sets the motor's velocity using PID
     *
     * @param desiredVelocity desired velocity of the motor in ticks/100ms
     */
    public void setDesiredVelocity(double desiredVelocity) {
        if (!overridden)
            motor.set(ControlMode.Velocity, desiredVelocity);
    }


    /**
     * @param power to be set to the motor (between -1 and 1)
     */
    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }

    /**
     * @return the velocity of the motor in ticks/100ms
     */
    @Log(name = "Trigger/Motor Velocity")
    @Override
    public double getVelocity() {
        return motor.getSelectedSensorVelocity();
    }

    /**
     * @return the position of the motor in an array
     */
    public double[] getValues() {
        return new double[]{motor.getSelectedSensorPosition()};
    }

    @Override
    public void periodic() {
        motor.tunePID("Loader/PID");

    }
   
    @Override
    public String configureLogName() {
        return "Loader";
    }
}
