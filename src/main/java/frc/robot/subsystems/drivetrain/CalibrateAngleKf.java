package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.FeedforwardConstants;
import frc.robot.utilities.Logger;

public class CalibrateAngleKf extends CommandBase {
	private DrivetrainSS drivetrain;
	private double outputVoltage;
	private double endVoltage;
	private boolean postTest[];
	private FeedforwardConstants[] constants;
	private double lastVelocity[];
	private double VelocitySum[];
	private int sampleCount[];
	private Logger logger;
	private double discoveredVelocity[];

	private final static double CODE_ITERATION_RATE_IN_SECONDS = 0.02;

	/**
	 * * This command autonomously runs tests and outputs a csv file including the
	 * Kf value at different velocities this is used to find the optimal kF coefs
	 * for calculate the correct voltage for a given velocity in RPM
	 */
	public CalibrateAngleKf(DrivetrainSS drivetrain) {
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
		postTest = new boolean[4];
		constants = new FeedforwardConstants[4];
		lastVelocity = new double[4];
		VelocitySum = new double[4];
		sampleCount = new int[4];
		discoveredVelocity = new double[4];

		constants = drivetrain.getAngleFeedforwardConstants();
		outputVoltage = constants[0].initialOutput;
		endVoltage = outputVoltage + constants[0].accelerationPerTest * constants[0].testAmount;
		for (int i = 0; i < 4; i++) {
			this.constants[i] = constants[i];
			postTest[i] = false;
		}
	}

	@Override
	public void initialize() {
		this.logger = new Logger("Angle Swerve KF Calibration", "Voltage", "Velocity - Front left",
				"Velocity - front right", "Velocity - rear right", "Velocity - rear left");
		for (int i = 0; i < 4; i++) {
			postTest[i] = false;
			VelocitySum[i] = 0;
			sampleCount[i] = 0;
		}
		outputVoltage = constants[0].initialOutput;
	}

	@Override
	public void execute() {
		boolean allPostTest = true;
		double currentVelocity[] = new double[4];

		drivetrain.setAngleVoltage(outputVoltage);
		SmartDashboard.putNumber("voltage", outputVoltage);
		for (int i = 0; i < 4; i++) {
			SmartDashboard.putBoolean(i + " postTest", postTest[i]);
			currentVelocity[i] = drivetrain.getAngleMotorAPS()[i];
			SmartDashboard.putNumber(i + " current Vel", currentVelocity[i]);
			if (!postTest[i])
				allPostTest = false;
		}
		if (allPostTest) {
			outputVoltage += constants[0].accelerationPerTest;

			logger.log(outputVoltage, discoveredVelocity[0], discoveredVelocity[1], discoveredVelocity[2],
					discoveredVelocity[3]);
			for (int i = 0; i < 4; i++) {
				postTest[i] = false;
				discoveredVelocity[i] = 0;
				VelocitySum[i] = 0;
				sampleCount[i] = 0;
			}
		} else {
			drivetrain.setAngleVoltage(outputVoltage);
			for (int i = 0; i < 4; i++) {
				if (Math.abs(lastVelocity[i] - currentVelocity[i])
						/ CODE_ITERATION_RATE_IN_SECONDS < constants[i].accelerationTolerance) {
					VelocitySum[i] += currentVelocity[i];
					sampleCount[i]++;
				} else {
					VelocitySum[i] = 0;
					sampleCount[i] = 0;
				}

				if (sampleCount[i] == constants[i].sampleAmount) {
					double vel = VelocitySum[i] / sampleCount[i];
					discoveredVelocity[i] = vel;
					postTest[i] = true;
				}
			}
		}
		for (int i = 0; i < 4; i++) {
			lastVelocity[i] = currentVelocity[i];
		}
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.setAngleVoltage(0);
		logger.close();
	}

	@Override
	public boolean isFinished() {
		return outputVoltage >= endVoltage;
	}
}
