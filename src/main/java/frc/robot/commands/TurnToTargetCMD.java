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
public class TurnToTargetCMD extends CommandBase {
    private VanillaLimelight limelight;
    private VisionConstants visionConstants;
    private DrivetrainSS drivetrain;
    private Target target;
    private TrigonPIDController rotationPIDController;
    private double lastTimeSeenTarget;

    public TurnToTargetCMD(DrivetrainSS drivetrain, VanillaLimelight limelight, VisionConstants visionConstants,
                           Target target) {

        addRequirements(drivetrain);

        this.limelight = limelight;
        this.visionConstants = visionConstants;
        this.target = target;
        this.drivetrain = drivetrain;
        rotationPIDController = new TrigonPIDController(visionConstants.ROTATION_SETTINGS);
        SmartDashboard.putData("TurnToTargetCMD/rotation PID", rotationPIDController);
    }

    @Override
    public void initialize() {
        rotationPIDController.reset();
        SmartDashboard.putNumber("TurnToTargetCMD/Angle setpoint", SmartDashboard.getNumber("TurnToTargetCMD/Angle setpoint", 0));
        rotationPIDController.setSetpoint(0);
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        // Configure the limelight to start computing vision.
        limelight.startVision(target);
    }

    @Override
    public void execute() {
        rotationPIDController.setSetpoint(SmartDashboard.getNumber("TurnToTargetCMD/Angle setpoint", 0));
        if (limelight.hasTarget()) {
            double pid = -rotationPIDController.calculate(limelight.getTx());
            if (pid != 0)
                drivetrain.powerDrive(0, 0, pid);
            else
                drivetrain.stopDrive();
            lastTimeSeenTarget = Timer.getFPGATimestamp();
        } else {
            // The target wasn't found
            drivetrain.stopDrive();
            DriverStationLogger.logToDS("turnToTargetCMD: Target not found!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
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
