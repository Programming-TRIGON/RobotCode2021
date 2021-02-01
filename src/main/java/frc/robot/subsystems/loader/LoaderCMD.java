package frc.robot.subsystems.loader;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants;

public class LoaderCMD extends CommandBase {
  private LoaderSS loaderSS;
  private RobotConstants.LoaderConstants constants;
  private DoubleSupplier velocity;

  public LoaderCMD(LoaderSS loaderSS, RobotConstants.LoaderConstants constants, DoubleSupplier velocity) {
    this.loaderSS = loaderSS;
    this.constants = constants;
    this.velocity = velocity;
    addRequirements(loaderSS);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    loaderSS.setVelocity(velocity.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
