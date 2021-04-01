package frc.robot.subsystems.pitcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonDoubleSolenoid;
import frc.robot.constants.RobotConstants.PitcherConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class PitcherSS extends SubsystemBase implements Loggable {
    private final PitcherConstants constants;
    private final TrigonDoubleSolenoid rightSolenoid;
    private final TrigonDoubleSolenoid leftSolenoid;
    private final TrigonDoubleSolenoid masterSolenoid;

    public PitcherSS(PitcherConstants constants) {
        this.constants = constants;
        rightSolenoid = constants.PCM_MAP.RIGHT_SOLENOID;
        leftSolenoid = constants.PCM_MAP.LEFT_SOLENOID;
        masterSolenoid = rightSolenoid;
    }

    /**
     * Sets the state of the rightSolenoid with a boolean
     *
     * @param state to be set to the rightSolenoid (true=forward false=reverse)
     */
    public void setSolenoidState(boolean state) {
        rightSolenoid.setSolenoid(state);
        leftSolenoid.setSolenoid(state);
    }

    /**
     * Gets the state of the rightSolenoid as a boolean
     *
     * @return the state of the rightSolenoid (true=forward false=reverse)
     */
    @Log(name = "Is open")
    public boolean getSolenoidState() {
        return masterSolenoid.isForward();
    }

    public void toggleSolenoid() {
        setSolenoidState(!getSolenoidState());
    }

    @Override
    public String configureLogName() {
        return "Pitcher";
    }
}
