package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.led.LedSS;
import frc.robot.utilities.DriverStationLogger;
import frc.robot.vision.Limelight;
import frc.robot.vision.Target;

public class ShooterCMD extends CommandBase {
    public final ShooterSS shooterSS;
    private final ShooterConstants constants;
    private final Limelight limelight;
    private final LedSS ledSS;
    private double desiredVelocity;

    public ShooterCMD(ShooterSS shooterSS, ShooterConstants constants, Limelight limelight, LedSS ledSS) {
        this.shooterSS = shooterSS;
        this.constants = constants;
        this.limelight = limelight;
        this.ledSS = ledSS;
        addRequirements(shooterSS);
    }

    /**
     * Calculates the desired desiredVelocity to set the motors based on the height at which the limelight sees the target
     *
     * @return the desired desiredVelocity of the motors
     */
    //TODO: Set correct calculation based on function chosen for calculation.
    public double calculateVelocity() {
        double y = limelight.getTy();
        return constants.LIMELIGHT_VELOCITY_COEF_A * Math.pow(y, 2) + constants.LIMELIGHT_VELOCITY_COEF_B * y
                + constants.LIMELIGHT_VELOCITY_COEF_C;
    }

    @Override
    public void initialize() {
        limelight.setPipeline(limelight.getLimelightConstants().POWER_PORT_PIPELINE);
        limelight.startVision(Target.RocketMiddle);
        desiredVelocity = calculateVelocity();
    }

    @Override
    public void execute() {
        if (limelight.getTv()) {
            shooterSS.setDesiredVelocity(desiredVelocity);
            ledSS.setColor(ledSS.getColorMap().SHOOTER_ENABLED);
        }
        else {
            ledSS.blinkColor(ledSS.getColorMap().NO_TARGET);
            DriverStationLogger.logToDS("No valid target, Try reposition!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSS.stopMoving();
        ledSS.turnOffLED();
        limelight.stopVision();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
