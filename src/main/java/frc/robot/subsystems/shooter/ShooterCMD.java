package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.components.TBHController;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.led.LedSS;
import frc.robot.utilities.DriverStationLogger;
import frc.robot.utilities.TrigonPIDController;
import frc.robot.vision.Limelight;
import frc.robot.vision.Target;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.DoubleSupplier;

public class ShooterCMD extends CommandBase implements Loggable {
    public final ShooterSS shooterSS;
    private final ShooterConstants constants;
    private Limelight limelight;
    private final LedSS ledSS;
    private final TBHController TBHController;
    private final TrigonPIDController PIDController;
    private final SimpleMotorFeedforward simpleMotorFeedforward;
    private ShooterState currentState;
    @Log(name = "Shooter/Desired Velocity")
    private DoubleSupplier desiredVelocity;
    private final boolean isUsingLimelight;
    private double lastVelocity;
    private int ballsShotCount;

    private ShooterCMD(ShooterSS shooterSS, ShooterConstants constants, LedSS ledSS, boolean isUsingLimelight) {
        this.shooterSS = shooterSS;
        this.constants = constants;
        this.ledSS = ledSS;
        this.isUsingLimelight = isUsingLimelight;
        TBHController = constants.TBH_CONTROLLER;
        PIDController = constants.PID_CONTROLLER;
        this.simpleMotorFeedforward = constants.SIMPLE_MOTOR_FEEDFORWARD;
        addRequirements(shooterSS);
    }

    public ShooterCMD(ShooterSS shooterSS, ShooterConstants constants, LedSS ledSS, Limelight limelight) {
        this(shooterSS, constants, ledSS, true);
        this.limelight = limelight;
        this.desiredVelocity = this::calculateDesiredVelocity;
    }

    public ShooterCMD(ShooterSS shooterSS, ShooterConstants constants, LedSS ledSS, DoubleSupplier desiredVelocity) {
        this(shooterSS, constants, ledSS, false);
        this.desiredVelocity = desiredVelocity;
    }

    /**
     * Calculates the desired desiredVelocity to set the motors based on the height at which the limelight sees the target
     *
     * @return the desired desiredVelocity of the motors
     */
    //TODO: Set correct calculation based on function chosen for calculation.
    public double calculateDesiredVelocity() {
        double y = limelight.getTy();
        return constants.LIMELIGHT_VELOCITY_COEF_A * Math.pow(y, 2) + constants.LIMELIGHT_VELOCITY_COEF_B * y
                + constants.LIMELIGHT_VELOCITY_COEF_C;
    }

    @Override
    public void initialize() {
        currentState = ShooterState.AfterShot;
        ballsShotCount = 0;
        TBHController.reset();
        PIDController.reset();
        if (isUsingLimelight) {
            if (limelight.getTv()) {
                limelight.startVision(Target.PowerPort);
            }
            else {
                ledSS.blinkColor(ledSS.getColorMap().NO_TARGET);
                DriverStationLogger.logToDS("No valid target, Try reposition!");
            }
        }
    }

    @Override
    public void execute() {
        TBHController.setSetpoint(desiredVelocity.getAsDouble());
        PIDController.setSetpoint(desiredVelocity.getAsDouble());
        ledSS.setColor(ledSS.getColorMap().SHOOTER_ENABLED);

        // changes the state of the shooter based on if a ball was just shot and if the PIDF has gotten the velocity back to its target
        if (ballWasShot() && currentState == ShooterState.Default) {
            currentState = ShooterState.AfterShot;
            ballsShotCount++;
        }
        else if ((PIDController.atSetpoint() || PIDController.getPositionError() <= 0) && currentState == ShooterState.AfterShot) {
            currentState = ShooterState.Default;
            TBHController.setLastOutput(shooterSS.getPower());
        }

        // switches between using TBH and PIDF to control the velocity based on if a ball was just shot
        switch (currentState) {
            case Default:
                shooterSS.move(TBHController.calculate(shooterSS.getVelocity())
                        + simpleMotorFeedforward.calculate(shooterSS.getVelocity()));
                lastVelocity = shooterSS.getVelocity();
                break;
            case AfterShot:
                shooterSS.move(PIDController.calculate(shooterSS.getVelocity())
                        + simpleMotorFeedforward.calculate(shooterSS.getVelocity()));
                break;
        }
    }


    public boolean ballWasShot() {
        return lastVelocity - shooterSS.getVelocity() >= constants.BALL_SHOT_VELOCITY_DROP;
    }

    public int getBallsShotCount() {
        return ballsShotCount;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSS.stopMoving();
        ledSS.turnOffLED();
        if (isUsingLimelight)
            limelight.stopVision();
    }

    public enum ShooterState {
        Default, // default mode of the shooter, controls motors using TBH
        AfterShot; // mode for after a ball is shot and when first running the command, returns the shooter to target velocity using PIDF
    }
}
