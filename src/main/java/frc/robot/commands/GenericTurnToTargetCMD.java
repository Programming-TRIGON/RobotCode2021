package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.vision.Target;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.PIDCoefs;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is just template for a subsystem turn to target command. It will
 * probably be changed according the game and the robot.
 */
public class GenericTurnToTargetCMD extends CommandBase {
  private Limelight limelight;
  private RobotConstants.VisionConstants visionConstants;
  private MovableSubsystem subsystem;
  private Target target;
  private PIDController rotationPIDController;
  private double lastTimeSeenTarget;

  public GenericTurnToTargetCMD(Limelight limelight, RobotConstants.VisionConstants visionConstants, Target target,
      MovableSubsystem subsystem) {

    addRequirements(subsystem);

    this.limelight = limelight;
    this.visionConstants = visionConstants;
    this.target = target;
    this.subsystem = subsystem;

    PIDCoefs rotationSettings = visionConstants.ROTATION_SETTINGS;
    rotationPIDController = new PIDController(rotationSettings.getKP(), rotationSettings.getKI(),
        rotationSettings.getKD());
    rotationPIDController.setTolerance(rotationSettings.getTolerance(), rotationSettings.getDeltaTolerance());

  }

  @Override
  public void initialize() {
    rotationPIDController.reset();
    rotationPIDController.setSetpoint(0);
    lastTimeSeenTarget = Timer.getFPGATimestamp();
    // Configure the limelight to start computing vision.
    limelight.startVision(target);
  }

  @Override
  public void execute() {
    if (limelight.getTv()) {
      subsystem.move(rotationPIDController.calculate(limelight.getAngle()));
      lastTimeSeenTarget = Timer.getFPGATimestamp();
    } else
      // The target wasn't found
      subsystem.stopMoving();
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stopMoving();
    limelight.stopVision();
  }

  @Override
  public boolean isFinished() {
    return ((Timer.getFPGATimestamp() - lastTimeSeenTarget) > visionConstants.TARGET_TIME_OUT)
        || rotationPIDController.atSetpoint();
  }

  public void enableTuning() {
    SmartDashboard.putData("PID/visionRotation", rotationPIDController);
  }
}
