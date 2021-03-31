package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KFCallebratableSubsystem;
import frc.robot.utilities.FeedforwardConstants;
import frc.robot.utilities.Logger;

public class GenericCalibrateKF extends CommandBase {
    private final static int PIDF_MAX_OUTPUT = 1023;
    private final static double CODE_ITERATION_RATE = 0.02;

    private final KFCallebratableSubsystem subsystem;
    private final FeedforwardConstants constants;
    private double output;
    private double lastVelocity;
    private double velocitySum;
    private double KFSum;
    private int sampleCount;
    private int testCount;
    private boolean postTest;

    /**
     * This command autonomously runs tests and outputs the optimum KF value this is
     * used to calculate the correct voltage for a given velocity in RPM
     */
    public GenericCalibrateKF(KFCallebratableSubsystem subsystem, FeedforwardConstants constants) {
        this.subsystem = subsystem;
        this.constants = constants;
    }

    @Override
    public void initialize() {
        KFSum = 0;
        velocitySum = 0;
        sampleCount = 0;
        testCount = 0;
        output = constants.initialOutput;
        postTest = false;
        lastVelocity = subsystem.getVelocity();
    }

    @Override
    public void execute() {
        if (postTest) {
            subsystem.stopMoving();
            output += constants.accelerationPerTest;
            velocitySum = 0;
            sampleCount = 0;
            postTest = false;
        } else {
            subsystem.move(output);
            if (atSetpoint()) {
                velocitySum += subsystem.getVelocity();
                sampleCount++;

            } else {
                velocitySum = 0;
                sampleCount = 0;
            }
            if (sampleCount == constants.sampleAmount) {
                double averageVelocity = velocitySum / sampleCount;
                /*
                 * Uses calculation from Ctre to calculate the KF based on the given output and
                 * the velocity the motor got to (See CTRE Documentation for further explanation
                 * https://bit.ly/2QuAOFI")
                 */
                KFSum += (output * PIDF_MAX_OUTPUT) / averageVelocity;
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
        return Math.abs(subsystem.getVelocity() - lastVelocity) / CODE_ITERATION_RATE < constants.accelerationTolerance;
    }
}
