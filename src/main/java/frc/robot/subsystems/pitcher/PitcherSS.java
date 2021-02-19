package frc.robot.subsystems.pitcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonDoubleSolenoid;
import frc.robot.constants.RobotConstants.PitcherConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class PitcherSS extends SubsystemBase implements Loggable {
    private final TrigonDoubleSolenoid solenoid;
    private final PitcherConstants constants;

    public PitcherSS(PitcherConstants constants) {
        this.constants = constants;
        solenoid = constants.PCM_MAP.SOLENOID;
    }

    /**
     * Sets the position of the solenoid with a boolean
     *
     * @param position to be set to the solenoid (true=forward false=reverse)
     */
    public void setSolenoidState(boolean position) {
        solenoid.setSolenoid(position);
    }

    /**
     * Gets the position of the solenoid as a boolean
     *
     * @return the position of the solenoid (true=forward false=reverse)
     */
    @Log(name = "Solenoid state")
    public boolean getSolenoidState() {
        return solenoid.isForward();
    }
}
