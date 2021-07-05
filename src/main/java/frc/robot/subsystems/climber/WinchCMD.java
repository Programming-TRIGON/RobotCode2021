package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ClimberConstants;

public class WinchCMD extends CommandBase {
    private final WinchSS winchSS;
    private final ClimberConstants constants;
    private final DoubleSupplier power;

  public WinchCMD(WinchSS winchSS,ClimberConstants constants,DoubleSupplier power) {
      this.winchSS=winchSS;
      this.constants=constants;
      this.power=power;

      addRequirements(winchSS);
  }

  public WinchCMD(WinchSS liftSS,ClimberConstants constants){
    this(liftSS, constants,()-> constants.DEFAULT_WINCH_POWER);
  }

  @Override
  public void execute() {
    winchSS.move(power.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    winchSS.stopMoving();
  }
}
