package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ClimberConstants;

public class LiftCMD extends CommandBase {
    private final LiftSS liftSS;
    private final ClimberConstants constants;
    private final DoubleSupplier power;

  public LiftCMD(LiftSS liftSS,ClimberConstants constants,DoubleSupplier power) {
      this.liftSS=liftSS;
      this.constants=constants;
      this.power=power;

      addRequirements(liftSS);
  }

  public LiftCMD(LiftSS liftSS,ClimberConstants constants){
    this(liftSS, constants,()-> constants.DEFAULT_LIFT_POWER);
  }

  @Override
  public void execute() {
    liftSS.move(power.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    liftSS.stopMoving();
  }
}
