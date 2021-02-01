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
  private boolean hoodPosition;

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
      pitcherSS.setSolenoid(getIsHoodExtended() ? limelight.getTy() < constants.EXTENDED_TOGGLE_ANGLE
          : limelight.getTy() < constants.RETRACTED_TOGGLE_ANGLE);
      setIsHoodExtended(pitcherSS.getSolenoid());
    } else {
      ledSS.blinkColor(LedColor.Black, 5);
    }
  }
}
