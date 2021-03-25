package frc.robot.utilities;

public class FeedforwardConstants {
    public final double mCoef;
    public final double bCoef;
    public final double initialOutput;
    public final double accelerationPerTest;
    public final double accelerationTolerance;
    public final int sampleAmount;
    public final int testAmount;

    public FeedforwardConstants(double mCoef, double bCoef, double initialOutput, double accelerationPerTest,
                                double accelerationTolerance, int sampleAmount, int testAmount) {
        this.mCoef = mCoef;
        this.bCoef = bCoef;
        this.initialOutput = initialOutput;
        this.accelerationPerTest = accelerationPerTest;
        this.accelerationTolerance = accelerationTolerance;
        this.sampleAmount = sampleAmount;
        this.testAmount = testAmount;
    }

    public FeedforwardConstants(double mCoef, double bCoef, FeedforwardConstants feedforwardConstants) {
        this(mCoef, bCoef, feedforwardConstants.initialOutput, feedforwardConstants.accelerationPerTest,
                feedforwardConstants.accelerationTolerance, feedforwardConstants.sampleAmount,
                feedforwardConstants.testAmount);
    }

    public FeedforwardConstants(double initialOutput, double accelerationPerTest,
                                double accelerationTolerance, int sampleAmount, int testAmount) {
        this(0, 0, initialOutput, accelerationPerTest,
                accelerationTolerance, sampleAmount, testAmount);
    }
}
