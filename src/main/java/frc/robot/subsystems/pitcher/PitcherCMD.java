package frc.robot.subsystems.pitcher;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.PitcherConstants;
import frc.robot.subsystems.led.LedSS;
import frc.robot.vision.limelights.PitcherLimelight;

public class PitcherCMD extends CommandBase {
    private final PitcherSS pitcherSS;
    private final PitcherConstants constants;
    private final PitcherLimelight limelight;
    private final LedSS ledSS;
    private boolean hoodPosition;

    public PitcherCMD(PitcherSS pitcherSS, LedSS ledSS, PitcherConstants constants, PitcherLimelight limelight) {
        this.pitcherSS = pitcherSS;
        this.ledSS = ledSS;
        this.constants = constants;
        this.limelight = limelight;
        addRequirements(pitcherSS);
    }

    @Override
    public void initialize() {
        togglePitcherBasedOnDistance();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * sets the position of the limelight based on whether or not the shooter hood
     * is extended or retracted
     *
     * @param position of the limelight (true=extended false=retracted)
     */
    public void setIsHoodExtended(boolean position) {
        hoodPosition = position;
    }

    /**
     * gets the position of the limelight based on whether or not the shooter hood
     * is extended or retracted
     *
     * @return the current position of the limelight (true=extended false=retracted)
     */
    public boolean getIsHoodExtended() {
        return hoodPosition;
    }

    /**
     * Toggles the pitcher based on the current position of the limelight (extended
     * or retracted) and the current angle at which the limelight sees the target.
     */
    public void togglePitcherBasedOnDistance() {
        if (limelight.getTv()) {
            pitcherSS.setSolenoidState(getIsHoodExtended() ? limelight.getTy() < constants.EXTENDED_TOGGLE_ANGLE
                    : limelight.getTy() < constants.RETRACTED_TOGGLE_ANGLE);
            setIsHoodExtended(pitcherSS.getSolenoidState());
        }
        else {
            ledSS.blinkColor(ledSS.getColorMap().NO_TARGET);
        }
    }
}
