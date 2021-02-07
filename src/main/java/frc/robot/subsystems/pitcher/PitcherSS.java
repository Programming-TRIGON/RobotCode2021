package frc.robot.subsystems.pitcher;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonDoubleSolenoid;
import frc.robot.constants.RobotConstants;
import frc.robot.vision.Limelight;
import io.github.oblarg.oblog.Loggable;

public class PitcherSS extends SubsystemBase implements Loggable {
    private TrigonDoubleSolenoid solenoid;
    private RobotConstants.PitcherConstants constants;
    private Limelight limelight;

    public PitcherSS(RobotConstants.PitcherConstants constants, Limelight limelight) {
        this.constants = constants;
        this.limelight = limelight;
        this.solenoid = constants.PCM_MAP.SOLENOID;
    }

    /**
     * Sets the position of the solenoid wit a boolean
     *
     * @param position to be set to the solenoid (true=forward false=reverse)
     */
    public void setSolenoid(boolean position) {
        solenoid.setSolenoid(position);
    }

    /**
     * Gets the position of the solenoid as a boolean
     *
     * @return the position of the solenoid (true=forward false=reverse)
     */
    public boolean getSolenoid() {
        return solenoid.isForward();
    }

    /**
     * Toggles the pitcher based on the current position of the limelight (extended
     * or retracted) and the current angle at which the limelight sees the target.
     */
    public void togglePitcherBasedOnDistance() {
        if (limelight.getTv()) {
            solenoid.setSolenoid(limelight.getIsHoodExtended() ? limelight.getTy() < constants.EXTENDED_TOGGLE_ANGLE
                    : limelight.getTy() < constants.RETRACTED_TOGGLE_ANGLE);
            limelight.setIsHoodExtended(solenoid.isForward());
        }
    }
}
