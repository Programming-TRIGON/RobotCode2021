package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.led.LedSS;
import frc.robot.vision.Limelight;
import org.opencv.core.Mat;

public class ShooterCMD extends CommandBase {
    public final ShooterSS shooterSS;
    private final RobotConstants.ShooterConstants constants;
    private final Limelight limelight;
    private final RobotConstants.LimelightConstants limelightConstants;
    private final LedSS ledSS;
    private final RobotConstants.LedConstants.ColorMap colorMap;
    private double velocity;

    public ShooterCMD(ShooterSS shooterSS, RobotConstants.ShooterConstants constants, RobotConstants.LimelightConstants limelightConstants,
                      Limelight limelight, LedSS ledSS) {
        this.shooterSS = shooterSS;
        this.constants = constants;
        this.limelight = limelight;
        this.limelightConstants = limelightConstants;
        this.ledSS = ledSS;
        this.colorMap = ledSS.getColorMap();
        addRequirements(shooterSS);
    }

    /**
     * Calculates the desired velocity to set the motors based on the height at which the limelight sees the target
     *
     * @return the desired velocity of the motors
     */
    //TODO: Set correct calculation based on function chosen for calculation.
    public double calculateVelocity() {
        double y = limelight.getTy();
        return constants.LIMELIGHT_VELOCITY_COEF_A * Math.pow(y, 2) + constants.LIMELIGHT_VELOCITY_COEF_B * y
                + constants.LIMELIGHT_VELOCITY_COEF_C;
    }

    @Override
    public void initialize() {
        limelight.setPipeline(limelightConstants.SHOOTER_PIPELINE);
        velocity = calculateVelocity();
    }

    @Override
    public void execute() {
        shooterSS.setVelocity(velocity);
        ledSS.setColor(colorMap.SHOOTER_ENABLED);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSS.stopMoving();
        ledSS.turnOffLED();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
