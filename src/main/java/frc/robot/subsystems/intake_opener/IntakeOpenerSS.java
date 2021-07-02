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
        this.solenoid = constants.PCM_MAP.SOLENOID;
        this.constants = constants;
        solenoid.setSolenoid(false);
    }

    /**
     * Sets the state of the solenoid with a boolean
     *
     * @param isOpen to be set to the solenoid (true=forward false=reverse)
     */
    public void setSolenoidState(boolean isOpen) {
        // solenoid.setSolenoid(isOpen);
    }

    /**
     * Gets the state of the solenoid as a boolean
     *
     * @return the state of the solenoid (true=forward false=reverse)
     */
    @Log(name = "Is open")
    public boolean getSolenoidState() {
        return solenoid.isForward();
    }

    public void toggleSolenoid() {
        // solenoid.toggle();
    }

    @Override
    public String configureLogName() {
        return "Intake Opener";
    }
}
