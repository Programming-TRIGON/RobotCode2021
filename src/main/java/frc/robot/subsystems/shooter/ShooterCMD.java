package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
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
    private final LedSS ledSS;
    private final TBHController TBHController;
    private final TrigonPIDController PIDController;
    private final boolean isUsingLimelight;
    private PitcherLimelight limelight;
    private ShooterState currentState;
    @Log(name = "Shooter/Desired Velocity")
    private DoubleSupplier desiredVelocitySupplier;
    private double desiredVelocity;
    private double f;
    private double outputSum;
    private int ballsShotCount;
    private int sampleCount;
    private double lastTimeAtSetpoint;

    private ShooterCMD(ShooterSS shooterSS, LedSS ledSS, ShooterConstants constants, boolean isUsingLimelight) {
        this.shooterSS = shooterSS;
        this.constants = constants;
        this.ledSS = ledSS;
        this.isUsingLimelight = isUsingLimelight;
        TBHController = constants.TBH_CONTROLLER;
        PIDController = constants.PID_CONTROLLER;
        SmartDashboard.putData("Shooter/TBH", TBHController);
        SmartDashboard.putData("Shooter/PID", PIDController);
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
        currentState = ShooterState.PRETBH;
        ballsShotCount = 0;
        sampleCount = 0;
        outputSum = 0;
        lastTimeAtSetpoint = Timer.getFPGATimestamp();
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
            if (limelight.hasTarget())
                Shoot();
            else {
                if (ledSS != null)
                    ledSS.blinkColor(ledSS.getColorMap().NO_TARGET);
                DriverStationLogger.logToDS("ShooterCMD: No valid target, Try reposition!");
            }
        } else
            Shoot();
    }

    private void Shoot() {
        System.out.println(currentState.name());
        double output;
        TBHController.setSetpoint(desiredVelocity);
        PIDController.setSetpoint(desiredVelocity);
        if (ledSS != null)
            ledSS.setColor(ledSS.getColorMap().SHOOTER_ENABLED);

        // switches between using TBH and PIDF to control the velocity based on if a
        // ball was just shot
        switch (currentState) {
            case PRETBH:
                output = PIDController.calculate(shooterSS.getVelocityRPM()) + f;
                shooterSS.move(output);
                if (Math.abs(desiredVelocity - shooterSS.getVelocityRPM()) < TBHController.getTolerance()) {
                    TBHController.reset();
                    currentState = ShooterState.TBH;
                }
                System.out.println("pre" + shooterSS.getVelocityRPM() + "skncvsdkv: " + PIDController.calculate(shooterSS.getVelocityRPM()) + f);
                break;
            case TBH:
                output = TBHController.calculate(shooterSS.getVelocityRPM()) + f;
                shooterSS.move(output);
                if (Math.abs(desiredVelocity - shooterSS.getVelocityRPM()) < constants.TOLERANCE) {
                    outputSum += output;
                    sampleCount++;
                } else {
                    outputSum = 0;
                    sampleCount = 0;
                }
                if (ballsShotCount == 0 && sampleCount == constants.KF_CALCULATION_SAMPLE_AMOUNT) {
                    f = outputSum / sampleCount;
                    PIDController.reset();
                    currentState = ShooterState.PID;
                }
                CalculateIfAtSetpoint();
                SmartDashboard.putBoolean("isPID", false);
                System.out.println("TBH" + shooterSS.getVelocityRPM() + " s;mjvuisj: " + TBHController.calculate(shooterSS.getVelocityRPM()) + f);
                break;
            case PID:
                output = PIDController.calculate(shooterSS.getVelocityRPM()) + f;
                shooterSS.move(output);
                CalculateIfAtSetpoint();
                SmartDashboard.putBoolean("isPID", true);
                System.out.println("PID" + shooterSS.getVelocityRPM() + " seiknes: " + PIDController.calculate(shooterSS.getVelocityRPM()) + f);
                break;
        }
    }

    public int getBallsShotCount() {
        return ballsShotCount;
    }

    /**
     * Sets the lastTimeAtSetpoint to be equals to the current time if it is not on target.
     * This is used for calculating if the setpoint has been correct for a specified amount of time.
     * This should be called before setting the last velocity to be equal to the current velocity
     */
    private void CalculateIfAtSetpoint() {
        if (Math.abs(desiredVelocity - shooterSS.getVelocityRPM()) > constants.TOLERANCE)
            lastTimeAtSetpoint = Timer.getFPGATimestamp();

        SmartDashboard.putNumber("Timer.getFPGATimestamp() - lastTimeAtSetpoint", Timer.getFPGATimestamp() - lastTimeAtSetpoint);
    }


    public boolean isAtSetpoint() {
        return Timer.getFPGATimestamp() - lastTimeAtSetpoint > constants.TIME_AT_SETPOINT && currentState == ShooterState.PID;
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
        PRETBH,
        // default mode of the shooter, controls motors using TBH
        TBH,
        // mode for after a ball is shot and when first running the command,
        // returns the shooter to target velocity using PIDF
        PID;
    }
}
