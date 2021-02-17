package frc.robot.subsystems.pitcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonDoubleSolenoid;
import frc.robot.constants.RobotConstants.PitcherConstants;
import frc.robot.vision.Limelight;
import frc.robot.vision.PitcherLimelight;
import io.github.oblarg.oblog.Loggable;

public class PitcherSS extends SubsystemBase implements Loggable {
    private TrigonDoubleSolenoid solenoid;
    private PitcherConstants constants;
    private PitcherLimelight limelight;

    public PitcherSS(PitcherConstants constants, PitcherLimelight limelight) {
        this.constants = constants;
        this.limelight = limelight;
        this.solenoid = constants.PCM_MAP.SOLENOID;
    }

    /**
     * Sets the position of the solenoid with a boolean
     *
     * @param position to be set to the solenoid (true=forward false=reverse)
     */
    public void setSolenoidPosition(boolean position) {
        solenoid.setSolenoid(position);
    }

    /**
     * Gets the position of the solenoid as a boolean
     *
     * @return the position of the solenoid (true=forward false=reverse)
     */
    public boolean getSolenoidPosition() {
        return solenoid.isForward();
    }
}
