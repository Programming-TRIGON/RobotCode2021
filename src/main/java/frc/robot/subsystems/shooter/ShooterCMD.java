package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.components.TBHController;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.led.LedSS;
import frc.robot.utilities.DriverStationLogger;
import frc.robot.utilities.TrigonPIDController;
import frc.robot.vision.Target;
import frc.robot.vision.limelights.PitcherLimelight;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.DoubleSupplier;

public class ShooterCMD extends CommandBase implements Loggable {
    public final ShooterSS shooterSS;
    private final ShooterConstants constants;
    private PitcherLimelight limelight;
    private final LedSS ledSS;
    private final TBHController TBHController;
    private final TrigonPIDController PIDController;
    private ShooterState currentState;
    @Log(name = "Shooter/Desired Velocity")
    private DoubleSupplier desiredVelocitySupplier;
    private double desiredVelocity;
    private final boolean isUsingLimelight;
    private double lastVelocity;
    private double f;
    private double outputSum;
    private int ballsShotCount;
    private int sampleCount;

    private ShooterCMD(ShooterSS shooterSS, LedSS ledSS, ShooterConstants constants, boolean isUsingLimelight) {
        this.shooterSS = shooterSS;
        this.constants = constants;
        this.ledSS = ledSS;
        this.isUsingLimelight = isUsingLimelight;
        TBHController = constants.TBH_CONTROLLER;
        PIDController = constants.PID_CONTROLLER;
        addRequirements(shooterSS);
    }

    public ShooterCMD(ShooterSS shooterSS, LedSS ledSS, ShooterConstants constants, PitcherLimelight limelight) {
        this(shooterSS, ledSS, constants, true);
        this.limelight = limelight;
        this.desiredVelocitySupplier = limelight::calculateDesiredShooterVelocity;
    }

    public ShooterCMD(ShooterSS shooterSS, LedSS ledSS, ShooterConstants constants, DoubleSupplier desiredVelocitySupplier) {
        this(shooterSS, ledSS, constants, false);
        this.desiredVelocitySupplier = desiredVelocitySupplier;
    }

    @Override
    public void initialize() {
        shooterSS.setRampRate(constants.SHOOTING_RAMP_RATE);
        currentState = ShooterState.AfterShot;
        ballsShotCount = 0;
        sampleCount = 0;
        outputSum = 0;
        lastVelocity = shooterSS.getVelocityRPM();
        TBHController.reset();
        PIDController.reset();
        if (isUsingLimelight)
            limelight.startVision(Target.PowerPort);
        desiredVelocity = desiredVelocitySupplier.getAsDouble();
        f = constants.KF_COEF_A * desiredVelocity + constants.KF_COEF_B;
    }

    @Override
    public void execute() {
        if (isUsingLimelight) {
            if (limelight.getTv())
                Shoot();
            else {
                if (ledSS != null)
                    ledSS.blinkColor(ledSS.getColorMap().NO_TARGET);
                DriverStationLogger.logToDS("ShooterCMD: No valid target, Try reposition!");
            }
        }
        else
            Shoot();
    }

    private void Shoot() {
        double output;
        TBHController.setSetpoint(desiredVelocity);
        PIDController.setSetpoint(desiredVelocity);

        if (ledSS != null)
            ledSS.setColor(ledSS.getColorMap().SHOOTER_ENABLED);

        // changes the state of the shooter based on if a ball was just shot and if the
        // PIDF has gotten the velocity back to its target
        if (desiredVelocity - shooterSS.getVelocityRPM() >= constants.PID_COEFS.getTolerance()
                && currentState == ShooterState.Default) {
            currentState = ShooterState.AfterShot;
            ballsShotCount++;
        }
        else if (desiredVelocity - shooterSS.getVelocityRPM() < constants.PID_COEFS.getTolerance()
                && currentState == ShooterState.AfterShot) {
            currentState = ShooterState.Default;
            TBHController.reset();
            TBHController.setOutput(PIDController.calculate(shooterSS.getVelocityRPM()));
        }

        // switches between using TBH and PIDF to control the velocity based on if a
        // ball was just shot
        switch (currentState) {
            case Default:
                output = TBHController.calculate(shooterSS.getVelocityRPM()) + f;
                shooterSS.move(output);
                if (ballsShotCount == 0 && atSetpoint()) {
                    outputSum += output;
                    sampleCount++;
                }
                else {
                    outputSum = 0;
                    sampleCount = 0;
                }
                if (ballsShotCount == 0 && sampleCount == constants.KF_CALCULATION_SAMPLE_AMOUNT) {
                    f = outputSum / sampleCount;
                    TBHController.reset();
                }
                lastVelocity = shooterSS.getVelocityRPM();
                SmartDashboard.putBoolean("isPID", false);
                break;
            case AfterShot:
                output = PIDController.calculate(shooterSS.getVelocityRPM()) + f;
                shooterSS.move(output);
                lastVelocity = shooterSS.getVelocityRPM();
                SmartDashboard.putBoolean("isPID", true);
                break;
        }
    }

    public int getBallsShotCount() {
        return ballsShotCount;
    }

    public boolean atSetpoint() {
        return Math.abs(desiredVelocity - shooterSS.getVelocityRPM()) < constants.TOLERANCE
                && shooterSS.getVelocityRPM() - lastVelocity < constants.DELTA_TOLERANCE;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSS.setRampRate(constants.RIGHT_MOTOR_CONFIG.getRampRate());
        shooterSS.stopMoving();
        if (ledSS != null)
            ledSS.turnOffLED();
        if (isUsingLimelight)
            limelight.stopVision();
    }

    public enum ShooterState {
        // default mode of the shooter, controls motors using TBH
        Default,
        // mode for after a ball is shot and when first running the command,
        // returns the shooter to target velocity using PIDF
        AfterShot;
    }
}
