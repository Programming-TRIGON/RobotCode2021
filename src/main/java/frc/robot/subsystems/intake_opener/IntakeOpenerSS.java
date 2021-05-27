package frc.robot.subsystems.intake_opener;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonDoubleSolenoid;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeOpenerSS extends SubsystemBase implements Loggable {
    private final TrigonDoubleSolenoid solenoid;
    private final IntakeOpenerConstants constants;

    public IntakeOpenerSS(IntakeOpenerConstants constants) {
        solenoid = constants.PCM_MAP.SOLENOID;
        this.constants = constants;
    }

    /**
     * Sets the state of the rightSolenoid with a boolean
     *
     * @param state to be set to the rightSolenoid (true=forward false=reverse)
     */
    public void setSolenoidState(boolean state) {
        solenoid.setSolenoid(state);
    }

    /**
     * Gets the state of the rightSolenoid as a boolean
     *
     * @return the state of the rightSolenoid (true=forward false=reverse)
     */
    @Log(name = "Is open")
    public boolean getSolenoidState() {
        return solenoid.isForward();
    }

    public void toggleSolenoid() {
        setSolenoidState(!getSolenoidState());
    }

    @Override
    public String configureLogName() {
        return "Intake Opener";
    }
}
