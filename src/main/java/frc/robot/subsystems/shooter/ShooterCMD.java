package frc.robot.subsystems.shooter;

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

public class ShooterCMD extends CommandBase implements Loggable {
    public final ShooterSS shooterSS;
    private final ShooterConstants constants;
    private final Limelight limelight;
    private final LedSS ledSS;
    private final TBHController TBHController;
    private final TrigonPIDController PIDController;
    private ShooterState currentState;
    private double desiredVelocity;
    private double lastVelocity;
    private int ballsShotCount;

    public ShooterCMD(ShooterSS shooterSS, ShooterConstants constants, Limelight limelight, LedSS ledSS) {
        this.shooterSS = shooterSS;
        this.constants = constants;
        this.limelight = limelight;
        this.ledSS = ledSS;
        TBHController = constants.TBH_CONTROLLER;
        PIDController = constants.PID_CONTROLLER;
        addRequirements(shooterSS);
    }

    /**
     * Calculates the desired desiredVelocity to set the motors based on the height at which the limelight sees the target
     *
     * @return the desired desiredVelocity of the motors
     */
    //TODO: Set correct calculation based on function chosen for calculation.
    @Log(name = "Shooter/Velocity Calculation")
    public double calculateDesiredVelocity() {
        double y = limelight.getTy();
        return constants.LIMELIGHT_VELOCITY_COEF_A * Math.pow(y, 2) + constants.LIMELIGHT_VELOCITY_COEF_B * y
                + constants.LIMELIGHT_VELOCITY_COEF_C;
    }

    @Override
    public void initialize() {
        limelight.startVision(Target.PowerPort);
        desiredVelocity = calculateDesiredVelocity();
        currentState = ShooterState.Default;
        TBHController.setSetpoint(desiredVelocity);
        PIDController.setSetpoint(desiredVelocity);
        ballsShotCount = 0;
        TBHController.reset();
        PIDController.reset();
    }

    @Override
    public void execute() {
        if (limelight.getTv()) {
            ledSS.setColor(ledSS.getColorMap().SHOOTER_ENABLED);

            // changes the state of the shooter based on if a ball was just shot and if the PIDF has gotten the velocity back to its target
            if (ballWasShot() && currentState == ShooterState.Default) {
                currentState = ShooterState.AfterShot;
                TBHController.setLastOutput(shooterSS.getPower());
                ballsShotCount++;
            }
            else if ((PIDController.atSetpoint() || PIDController.getPositionError() >= 0) && currentState == ShooterState.AfterShot)
                currentState = ShooterState.Default;

            // switches between using TBH and PIDF to control the velocity based on if a ball was just shot
            switch (currentState) {
                case Default:
                    shooterSS.move(TBHController.calculate(shooterSS.getVelocity()));
                    lastVelocity= shooterSS.getVelocity();
                    break;
                case AfterShot:
                    shooterSS.move(PIDController.calculateWithKF(shooterSS.getVelocity()));
                    break;
            }
        }
        else {
            ledSS.blinkColor(ledSS.getColorMap().NO_TARGET);
            DriverStationLogger.logToDS("No valid target, Try reposition!");
        }
    }

    public boolean ballWasShot() {
        return shooterSS.getVelocity() - lastVelocity < constants.BALL_SHOT_VELOCITY_DROP;
    }

    @Override
    public boolean isFinished() {
        return ballsShotCount >= constants.MAX_NUMBER_OF_BALLS;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSS.stopMoving();
        ledSS.turnOffLED();
        limelight.stopVision();
    }

    public enum ShooterState {
        Default, // default mode of the shooter, controls motors using TBH
        AfterShot; // mode for after a ball is shot, returns the shooter to target velocity using PIDF
    }
}
