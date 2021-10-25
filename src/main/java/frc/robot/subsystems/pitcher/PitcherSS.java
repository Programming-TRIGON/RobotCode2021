package frc.robot.subsystems.pitcher;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonDoubleSolenoid;
import frc.robot.constants.RobotConstants.PitcherConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class PitcherSS extends SubsystemBase implements Loggable {
    private final PitcherConstants constants;
    private final TrigonDoubleSolenoid solenoid;
    private boolean hoodExtended;

    public PitcherSS(PitcherConstants constants) {
        this.constants = constants;
        solenoid = constants.PCM_MAP.SOLENOID;
        hoodExtended = true;
    }

    /**
     * Sets the state of the solenoid with a boolean
     *
     * @param state to be set to the solenoid (true=forward false=reverse)
     */
    public void setSolenoidState(boolean state) {
        hoodExtended = state;
        solenoid.setSolenoid(state);
    }

    /**
     * Gets the state of the solenoid as a boolean
     *
     * @return the state of the solenoid (true=forward false=reverse)
     */
    @Log(name = "Is open")
    public boolean getSolenoidState() {
        // this boolean values is used instead of solinoid.isforward() because that function didn't work
        return hoodExtended;
    }

    public void toggleSolenoid() {
        solenoid.setSolenoid(!getSolenoidState());
        hoodExtended = !getSolenoidState();
    }

    @Override
    public String configureLogName() {
        return "Pitcher";
    }
}
