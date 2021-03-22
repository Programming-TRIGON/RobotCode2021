package frc.robot.subsystems.drivetrain;

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
	private double lastPosition[];
	private double VelocitySum[];
	private int sampleCount[];
	private Logger logger[];

	private final static double CODE_ITERATION_RATE_IN_SECONDS = 0.02;

	/**
	 * * This command autonomously runs tests and outputs a csv file including the
	 * Kf value at different velocities this is used to find the optimal kF coefs
	 * for calculate the correct voltage for a given velocity in RPM
	 */
	public CalibrateAngleKf(DrivetrainSS drivetrain, FeedforwardConstants constants[]) {
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);

		outputVoltage = constants[0].initialVoltage;
		endVoltage = outputVoltage + constants[0].accelerationPerTest * constants[0].testAmount;
		for (int i = 0; i < 4; i++) {
			this.constants[i] = constants[i];
			postTest[i] = false;
		}
	}

	@Override
	public void initialize() {
		for (int i = 0; i < 4; i++) {
			this.logger[i] = new Logger(" - Angle Swerve KF Calibration", "Velocity", "Voltage");
			postTest[i] = false;
			lastPosition[i] = drivetrain.getStates()[i].angle.getDegrees();
			VelocitySum[i] = 0;
			sampleCount[i] = 0;
		}
		outputVoltage = constants[0].initialVoltage;
	}

	@Override
	public void execute() {
		boolean allPostTest = true;
		double currentVelocity[] = new double[4];

		drivetrain.setAngleVoltage(outputVoltage);
		for (int i = 0; i < 4; i++) {
			currentVelocity[i] = drivetrain.getStates()[i].angle.getDegrees()
					- lastPosition[i] / CODE_ITERATION_RATE_IN_SECONDS;
			if (!postTest[i])
				allPostTest = false;
		}
		if (allPostTest) {
			outputVoltage += constants[0].accelerationPerTest;
			for (int i = 0; i < 4; i++) {
				postTest[i] = false;
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
					double f = VelocitySum[i] / sampleCount[i];
					logger[i].log(outputVoltage, f);
					postTest[i] = true;
				}
			}
		}
		for (int i = 0; i < 4; i++) {
			lastPosition[i] = drivetrain.getStates()[i].angle.getDegrees();
			lastVelocity[i] = currentVelocity[i];
		}
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.setAngleVoltage(0);
		for (int i = 0; i < 4; i++) {
			logger[i].close();
		}
	}

	@Override
	public boolean isFinished() {
		return outputVoltage >= endVoltage;
	}
}
