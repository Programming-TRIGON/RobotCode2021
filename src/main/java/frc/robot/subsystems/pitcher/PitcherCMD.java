package frc.robot.subsystems.pitcher;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.led.LedColor;
import frc.robot.subsystems.led.LedSS;
import frc.robot.vision.Limelight;

public class PitcherCMD extends CommandBase {
  private PitcherSS pitcherSS;
  private RobotConstants.PitcherConstants constants;
  private Limelight limelight;
  private LedSS ledSS;

  public PitcherCMD(PitcherSS pitcherSS, RobotConstants.PitcherConstants constants, Limelight limelight) {
    this.pitcherSS = pitcherSS;
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
   * Toggles the pitcher based on the current position of the limelight (extended
   * or retracted) and the current angle at which the limelight sees the target.
   */
  public void togglePitcherBasedOnDistance() {
    if (limelight.getTv()) {
      pitcherSS.setSolenoid(limelight.getIsHoodExtended() ? limelight.getTy() < constants.EXTENDED_TOGGLE_ANGLE
          : limelight.getTy() < constants.RETRACTED_TOGGLE_ANGLE);
      limelight.setIsHoodExtended(pitcherSS.getSolenoid());
    } else {
      ledSS.blinkColor(LedColor.Black, 5);
    }
  }
}
