package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants;

public class IntakeCMD extends CommandBase {
  private IntakeSS intakeSS;
  private RobotConstants.IntakeConstants constants;
  private DoubleSupplier power;

  public IntakeCMD(IntakeSS intakeSS, RobotConstants.IntakeConstants constants, DoubleSupplier power) {
    this.intakeSS = intakeSS;
    this.constants = constants;
    this.power = power;
    addRequirements(intakeSS);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeSS.move(power.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
