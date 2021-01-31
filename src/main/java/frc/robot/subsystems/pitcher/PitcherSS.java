package frc.robot.subsystems.pitcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonDoubleSolenoid;
import frc.robot.constants.RobotConstants;
import frc.robot.vision.Limelight;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

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
   * Toggles the pitcher based on the current position of the limelight (extended
   * or retracted) and the current angle at which the limelight sees the target.
   */
  @Log(name = "Pitcher/Pitcher Position")
  public void togglePitcherBasedOnDistance() {
    if (limelight.getLimelightPosition() == true) {
      if (limelight.getTy() < constants.EXTENDED_TOGGLE_ANGLE)
        solenoid.setSolenoid(true);
      else
        solenoid.setSolenoid(false);
    } else {
      if (limelight.getTy() < constants.RETRACTED_TOGGLE_ANGLE)
        solenoid.setSolenoid(true);
      else
        solenoid.setSolenoid(false);
    }
  }

}
