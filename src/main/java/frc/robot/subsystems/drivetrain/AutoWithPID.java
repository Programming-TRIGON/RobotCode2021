package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Auto;
import frc.robot.constants.RobotConstants.DrivetrainConstants;
import frc.robot.utilities.TrigonPIDController;

public class AutoWithPID extends CommandBase {
  private final Auto auto;
  private final DrivetrainSS drivetrain;
  private final DrivetrainConstants constants;
  private final TrigonPIDController xPID;
  private final TrigonPIDController yPID;
  private final TrigonPIDController zPID;

  /** Creates a new DriveWithPID. */
  public AutoWithPID(Auto auto, DrivetrainSS drivetrain, DrivetrainConstants constants) {
    addRequirements(drivetrain);
    this.auto = auto;
    this.drivetrain = drivetrain;
    this.constants = constants;
    xPID = new TrigonPIDController(constants.AUTO_SPEED_PIDF_COEFS);
    yPID = new TrigonPIDController(constants.AUTO_SPEED_PIDF_COEFS);
    zPID = new TrigonPIDController(constants.AUTO_ROTATION_PIDF_COEFS);
    xPID.setSetpoint(auto.xSetpoint);
    yPID.setSetpoint(auto.ySetpoint);
    zPID.setSetpoint(auto.zSetpoint);
  }

  @Override
  public void initialize() {
    drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(drivetrain.getAngle())));
    xPID.reset();
    yPID.reset();
    zPID.reset();
  }

  @Override
  public void execute() {
    drivetrain.fieldPowerDrive(xPID.calculate(drivetrain.getX()), xPID.calculate(drivetrain.getY()),
        zPID.calculate(drivetrain.getAngle()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
  }

  @Override
  public boolean isFinished() {
    return (xPID.atSetpoint() || !auto.requireXSetpoint) && (zPID.atSetpoint() || !auto.requireZSetpoint)
        && (zPID.atSetpoint() || !auto.requireZSetpoint);
  }
}
