package frc.robot.subsystems.intake_opener;

import edu.wpi.first.wpilibj.DoubleSolenoid;
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
    }

    /**
     * Sets the state of the solenoid with a boolean
     *
     * @param state to be set to the solenoid (true=forward false=reverse)
     */
    public void setSolenoidState(boolean state) {
        solenoid.setSolenoid(state);
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
        DoubleSolenoid.Value value = solenoid.get();

        if (value == DoubleSolenoid.Value.kForward) {
            solenoid.set(DoubleSolenoid.Value.kReverse);
        } else if (value == DoubleSolenoid.Value.kReverse) {
            solenoid.set(DoubleSolenoid.Value.kForward);
        } else if (value == DoubleSolenoid.Value.kOff) {
            solenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    @Override
    public String configureLogName() {
        return "Intake Opener";
    }
}
