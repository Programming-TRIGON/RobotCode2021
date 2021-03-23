package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KFCallebratableSubsystem;
import frc.robot.utilities.FeedforwardConstants;
import frc.robot.utilities.Logger;


public class GenericCalibrateKF extends CommandBase {
    private final KFCallebratableSubsystem subsystem;
    private final FeedforwardConstants constants;
    private Logger logger;
    private double power;
    private double endVelocity;
    private double lastVelocity;
    private double velocitySum;
    private int sampleCount;
    private boolean postTest;

    public GenericCalibrateKF(KFCallebratableSubsystem subsystem, FeedforwardConstants constants) {
        this.subsystem = subsystem;
        this.constants = constants;
    }

    @Override
    public void initialize() {
        this.logger = new Logger("KF Calibration.csv", "Power", "Average Velocity");
        power = constants.initialDesiredOutput;
        endVelocity = constants.initialDesiredOutput + (constants.accelerationPerTest * constants.testAmount);
        postTest = false;
        lastVelocity = subsystem.getVelocity();
    }

    @Override
    public void execute() {
        if (postTest) {
            subsystem.stopMoving();
            if (subsystem.getVelocity() == 0) {
                power += constants.accelerationPerTest;
                postTest = false;
            }
        }
        else {
            double output = power;
            subsystem.move(output);
            if (atSetpoint()) {
                velocitySum += subsystem.getVelocity();
                sampleCount++;

            }
            else {
                velocitySum = 0;
                sampleCount = 0;
            }
            if (sampleCount == constants.sampleAmount) {
                double averageVelocity = velocitySum / sampleCount;
                logger.log(output, averageVelocity);
                postTest = true;
            }
        }

        lastVelocity = subsystem.getVelocity();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

    public boolean atSetpoint() {
        return Math.abs(power - subsystem.getVelocity()) < constants.accelerationTolerance;
    }
}
