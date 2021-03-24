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
    private double KFSum;
    private int sampleCount;
    private int testCount;
    private boolean postTest;

    public GenericCalibrateKF(KFCallebratableSubsystem subsystem, FeedforwardConstants constants) {
        this.subsystem = subsystem;
        this.constants = constants;
    }

    @Override
    public void initialize() {
        KFSum = 0;
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
            subsystem.move(power);
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
                KFSum += (power * 1023) / averageVelocity;
                testCount++;
                postTest = true;
            }
        }

        lastVelocity = subsystem.getVelocity();
    }

    @Override
    public boolean isFinished() {
        return testCount >= constants.testAmount;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMoving();
        System.out.println(KFSum / testCount);
    }

    public boolean atSetpoint() {
        return Math.abs(subsystem.getVelocity() - lastVelocity) / 0.02 < constants.accelerationTolerance;
    }
}
