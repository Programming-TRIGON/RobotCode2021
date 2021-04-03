package frc.robot.subsystems.pitcher;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.RobotConstants.PitcherConstants;
import frc.robot.subsystems.led.LedSS;
import frc.robot.utilities.DriverStationLogger;
import frc.robot.vision.limelights.PitcherLimelight;

public class PitcherCMD extends CommandBase {
    private final PitcherSS pitcherSS;
    private final PitcherConstants constants;
    private final PitcherLimelight limelight;
    private final LedSS ledSS;

    public PitcherCMD(PitcherSS pitcherSS, LedSS ledSS, PitcherConstants constants, PitcherLimelight limelight) {
        this.pitcherSS = pitcherSS;
        this.ledSS = ledSS;
        this.constants = constants;
        this.limelight = limelight;
        addRequirements(pitcherSS);
    }

    /**
     * Toggles the pitcher based on the current position of the limelight (extended
     * or retracted) and the current angle at which the limelight sees the target.
     */
    @Override
    public void initialize() {
        System.out.println("ptchr");
        if (limelight.hasTarget()) {
            pitcherSS.setSolenoidState(pitcherSS.getSolenoidState() ? limelight.getTy() > constants.EXTENDED_TOGGLE_ANGLE
                    : limelight.getTy() < constants.RETRACTED_TOGGLE_ANGLE);
        }
        else {
            if(ledSS != null)
                ledSS.blinkColor(ledSS.getColorMap().NO_TARGET);
            DriverStationLogger.logToDS("PitcherCMD: No Target Found");
        }
    }
}
