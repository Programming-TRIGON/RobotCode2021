package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.components.TBHController;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.led.LedSS;
import frc.robot.utilities.TrigonPIDController;

import java.util.function.DoubleSupplier;


public class ShooterWithoutLimelightCMD extends CommandBase {
    private final ShooterSS shooterSS;
    private final ShooterConstants constants;
    private final LedSS ledSS;
    private final TBHController TBHController;
    private final TrigonPIDController PIDController;
    private ShooterCMD.ShooterState currentState;
    private final double desiredVelocity;
    private double lastVelocity;
    private int ballsShotCount;

    public ShooterWithoutLimelightCMD(ShooterSS shooterSS, ShooterConstants constants, LedSS ledSS, DoubleSupplier desiredVelocity) {
        this.shooterSS = shooterSS;
        this.constants = constants;
        this.ledSS = ledSS;
        TBHController = constants.TBH_CONTROLLER;
        PIDController = constants.PID_CONTROLLER;
        this.desiredVelocity = desiredVelocity.getAsDouble();
        addRequirements(shooterSS);
    }

    public ShooterWithoutLimelightCMD(ShooterSS shooterSS, ShooterConstants constants, LedSS ledSS) {
        this(shooterSS, constants, ledSS, () -> constants.DEFAULT_DESIRED_VELOCITY);
    }

    @Override
    public void initialize() {
        TBHController.reset();
        TBHController.setSetpoint(desiredVelocity);
        PIDController.setSetpoint(desiredVelocity);
        ballsShotCount = 0;
    }

    @Override
    public void execute() {
        ledSS.setColor(ledSS.getColorMap().SHOOTER_ENABLED);

        // changes the state of the shooter based on if a ball was just shot and if the PIDF has gotten the velocity back to its target
        if (ballWasShot() && currentState == ShooterCMD.ShooterState.Default) {
            currentState = ShooterCMD.ShooterState.AfterShot;
            TBHController.setLastOutput(shooterSS.getPower());
            ballsShotCount++;
        }
        else if ((PIDController.atSetpoint() || PIDController.getPositionError() >= 0) && currentState == ShooterCMD.ShooterState.AfterShot)
            currentState = ShooterCMD.ShooterState.Default;

        // switches between using TBH and PIDF to control the velocity based on if a ball was just shot
        switch (currentState) {
            case Default:
                shooterSS.move(TBHController.calculate(shooterSS.getVelocity()));
                lastVelocity = shooterSS.getVelocity();
                break;
            case AfterShot:
                shooterSS.move(PIDController.calculateWithKF(shooterSS.getVelocity()));
                break;
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
    }

    public enum ShooterState {
        Default, // default mode of the shooter, controls motors using TBH
        AfterShot; // mode for after a ball is shot, returns the shooter to target velocity using PIDF
    }
}
