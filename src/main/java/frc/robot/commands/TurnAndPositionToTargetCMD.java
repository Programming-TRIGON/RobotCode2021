package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.VisionConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.utilities.DriverStationLogger;
import frc.robot.utilities.TrigonPIDController;
import frc.robot.vision.Target;
import frc.robot.vision.limelights.VanillaLimelight;

/**
 * Turns the drivetrain to face a given target
 */
public class TurnAndPositionToTargetCMD extends CommandBase {
    private VanillaLimelight limelight;
    private VisionConstants visionConstants;
    private DrivetrainSS drivetrain;
    private Target target;
    private TrigonPIDController rotationPIDController;
    private TrigonPIDController positionPIDController;
    private double lastTimeSeenTarget, yTarget;


    public TurnAndPositionToTargetCMD(DrivetrainSS drivetrain, VanillaLimelight limelight, VisionConstants visionConstants,
                                      Target target) {

        addRequirements(drivetrain);

        this.limelight = limelight;
        this.visionConstants = visionConstants;
        this.target = target;
        this.drivetrain = drivetrain;
        this.yTarget = visionConstants.Y_TARGET;
        rotationPIDController = new TrigonPIDController(visionConstants.ROTATION_SETTINGS);
        positionPIDController = new TrigonPIDController(visionConstants.POSITION_SETTINGS);

        SmartDashboard.putData("TurnAndPositionToTargetCMD/rotation PID", rotationPIDController);
        SmartDashboard.putData("TurnAndPositionToTargetCMD/position PID", positionPIDController);
        SmartDashboard.putNumber("TurnAndPositionToTargetCMD/y target", visionConstants.Y_TARGET);
    }

    @Override
    public void initialize() {
        rotationPIDController.reset();
        rotationPIDController.setSetpoint(0);
        positionPIDController.setSetpoint(SmartDashboard.getNumber("TurnAndPositionToTargetCMD/y target", visionConstants.Y_TARGET));
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        // Configure the limelight to start computing vision.
        limelight.startVision(target);
    }

    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            drivetrain.powerDrive(0, positionPIDController.calculate(limelight.getTy()), -rotationPIDController.calculate(limelight.getAngle()));
            lastTimeSeenTarget = Timer.getFPGATimestamp();
        } else {
            // The target wasn't found
            drivetrain.stopMoving();
            DriverStationLogger.logToDS("TurnAndPositionToTargetCMD: Target not found!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
        //	limelight.stopVision();
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
