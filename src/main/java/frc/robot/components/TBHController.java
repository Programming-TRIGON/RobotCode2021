package frc.robot.components;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpiutil.math.MathUtil;

public class TBHController implements Sendable {
    private double output;
    private double KI;
    private double setpoint;
    private double lastError;
    private double tbh;
    private double tolerance;
    private double lastOutput;

    public TBHController(double KI, double tolerance) {
        this.KI = KI;
        this.tolerance = tolerance;
        SendableRegistry.add(this, "TBHController");
    }

    public double calculate(double measurement) {
        double error = setpoint - measurement;
        output += KI * error;
        output = MathUtil.clamp(output, -1, 1);
        if (isPositive(error) != isPositive(lastError)) {
            tbh = lastOutput;
            output = 0.5 * (output + tbh);
        }
        lastError = error;
        lastOutput = output;
        return output;
    }

    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return calculate(measurement);
    }

    /**
     * Returns true if the error is within the percentage of the total input range, determined by
     * SetTolerance. This assumes that the maximum and minimum input were set using SetInput.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Is the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        return Math.abs(lastError) < tolerance;
    }

    /**
     * Resets the previous error and the integral term.
     */
    public void reset() {
        lastError = 0;
        output = 0;
        lastOutput = 0;
        tbh = 0;
    }

    public double getKI() {
        return KI;
    }

    public void setKI(double KI) {
        this.KI = KI;
    }

    /**
     * Sets the error which is considered acceptable for use with atSetpoint().
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setLastOutput(double output) {
        this.output = output;
        lastOutput = output;
    }

    private boolean isPositive(double value) {
        return value > 0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("TBHController");
        builder.addDoubleProperty("KI", this::getKI, this::setKI);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}
