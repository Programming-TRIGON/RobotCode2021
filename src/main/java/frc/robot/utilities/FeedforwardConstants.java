package frc.robot.utilities;

public class FeedforwardConstants {
	public final double mCoef;
	public final double bCoef;
	public final double initialVoltage;
	public final double accelerationPerTest;
	public final double accelerationTolerance;
	public final int sampleAmount;
	public final int testAmount;

	public FeedforwardConstants(double mCoef, double bCoef, double initialVoltage, double accelerationPerTest,
			double accelerationTolerance, int sampleAmount, int testAmount) {
		this.mCoef = mCoef;
		this.bCoef = bCoef;
		this.initialVoltage = initialVoltage;
		this.accelerationPerTest = accelerationPerTest;
		this.accelerationTolerance = accelerationTolerance;
		this.sampleAmount = sampleAmount;
		this.testAmount = testAmount;
	}

	public FeedforwardConstants(double aCoef, double bCoef, FeedforwardConstants feedforwardConstants) {
		this(aCoef, bCoef, feedforwardConstants.initialVoltage, feedforwardConstants.accelerationPerTest,
				feedforwardConstants.accelerationTolerance, feedforwardConstants.sampleAmount,
				feedforwardConstants.testAmount);
	}
}
