package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.FeedforwardConstants;
import frc.robot.utilities.Logger;

public class CalibrateSpeedKf extends CommandBase {
    private final static double CODE_ITERATION_RATE_IN_SECONDS = 0.02;
    private final DrivetrainSS drivetrain;
    private final double endVoltage;
    private final boolean[] postTest;
    private final double[] lastVelocity;
    private final double[] velocitySum;
    private final int[] sampleCount;
    private final double[] discoveredVelocity;
    private double outputVoltage;
    private FeedforwardConstants[] constants;
    private Logger logger;

    /**
     * * This command autonomously runs tests and outputs a csv file including the
     * Kf value at different velocities this is used to find the optimal kF coefs
     * for calculate the correct voltage for a given velocity in RPM
     */
    public CalibrateSpeedKf(DrivetrainSS drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        postTest = new boolean[4];
        constants = new FeedforwardConstants[4];
        lastVelocity = new double[4];
        velocitySum = new double[4];
        sampleCount = new int[4];
        discoveredVelocity = new double[4];

        constants = drivetrain.getSpeedFeedforwardConstants();
        outputVoltage = constants[0].initialOutput;
        endVoltage = outputVoltage + constants[0].accelerationPerTest * constants[0].testAmount;
        for (int i = 0; i < 4; i++) {
            this.constants[i] = constants[i];
            postTest[i] = false;
        }
    }

    @Override
    public void initialize() {
        this.logger = new Logger("Speed Swerve KF Calibration", "Voltage", "Velocity - Front left",
                "Velocity - front right", "Velocity - rear right", "Velocity - rear left");
        outputVoltage = constants[0].initialOutput;
    }

    @Override
    public void execute() {
        SwerveModuleState[] desiredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
            desiredStates[i] = new SwerveModuleState(0, new Rotation2d(0));
        drivetrain.setDesiredStates(desiredStates);
        boolean allPostTest = true;
        double[] currentVelocity = new double[4];

        drivetrain.setSpeedMotorsVoltage(outputVoltage);
        SmartDashboard.putNumber("voltage", outputVoltage);
        for (int i = 0; i < 4; i++) {
            SmartDashboard.putBoolean(i + " postTest", postTest[i]);
            currentVelocity[i] = drivetrain.getSpeedMotorsMPS(i);
            SmartDashboard.putNumber(i + " current Vel", currentVelocity[i]);
            if (allPostTest && !postTest[i])
                allPostTest = false;
        }
        if (allPostTest) {
            outputVoltage += constants[0].accelerationPerTest;

            logger.log(outputVoltage, discoveredVelocity[0], discoveredVelocity[1], discoveredVelocity[2],
                    discoveredVelocity[3]);
            for (int i = 0; i < 4; i++) {
                postTest[i] = false;
                discoveredVelocity[i] = 0;
                velocitySum[i] = 0;
                sampleCount[i] = 0;
            }
        } else {
            drivetrain.setSpeedMotorsVoltage(outputVoltage);
            for (int i = 0; i < 4; i++) {
                if (Math.abs(lastVelocity[i] - currentVelocity[i])
                        / CODE_ITERATION_RATE_IN_SECONDS < constants[i].accelerationTolerance) {
                    velocitySum[i] += currentVelocity[i];
                    sampleCount[i]++;
                } else {
                    velocitySum[i] = 0;
                    sampleCount[i] = 0;
                }

                if (sampleCount[i] == constants[i].sampleAmount) {
                    double vel = velocitySum[i] / sampleCount[i];
                    discoveredVelocity[i] = vel;
                    postTest[i] = true;
                }
            }
        }
        System.arraycopy(currentVelocity, 0, lastVelocity, 0, 4);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeedMotorsVoltage(0);
        for (int i = 0; i < 4; i++) {
            postTest[i] = false;
            velocitySum[i] = 0;
            sampleCount[i] = 0;
        }
        logger.close();
    }

    @Override
    public boolean isFinished() {
        return outputVoltage >= endVoltage;
    }
}
