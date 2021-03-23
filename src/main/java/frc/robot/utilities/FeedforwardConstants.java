package frc.robot.utilities;

public class FeedforwardConstants {
    public final double mCoef;
    public final double bCoef;
    public final double initialDesiredOutput;
    public final double accelerationPerTest;
    public final double accelerationTolerance;
    public final int sampleAmount;
    public final int testAmount;

    public FeedforwardConstants(double mCoef, double bCoef, double initialDesiredOutput, double accelerationPerTest,
                                double accelerationTolerance, int sampleAmount, int testAmount) {
        this.mCoef = mCoef;
        this.bCoef = bCoef;
        this.initialDesiredOutput = initialDesiredOutput;
        this.accelerationPerTest = accelerationPerTest;
        this.accelerationTolerance = accelerationTolerance;
        this.sampleAmount = sampleAmount;
        this.testAmount = testAmount;
    }

    public FeedforwardConstants(double mCoef, double bCoef, FeedforwardConstants feedforwardConstants) {
        this(mCoef, bCoef, feedforwardConstants.initialDesiredOutput, feedforwardConstants.accelerationPerTest,
                feedforwardConstants.accelerationTolerance, feedforwardConstants.sampleAmount,
                feedforwardConstants.testAmount);
    }

    public FeedforwardConstants(double initialDesiredOutput, double accelerationPerTest,
                                double accelerationTolerance, int sampleAmount, int testAmount) {
        this(0, 0, initialDesiredOutput, accelerationPerTest,
                accelerationTolerance, sampleAmount, testAmount);
    }
}
