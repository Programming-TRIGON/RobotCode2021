package frc.robot.utilities;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class TrigonPIDController extends PIDController {
    private double f;

    public TrigonPIDController(PIDCoefs pidCoefs) {
        super(pidCoefs.getKP(), pidCoefs.getKI(), pidCoefs.getKD());
        setTolerance(pidCoefs.getTolerance(), pidCoefs.getDeltaTolerance());
        f = pidCoefs.getKF();
    }

    /**
     * Get the Feed-Forward coefficient.
     *
     * @return Feed-Forward coefficient
     */
    public double getF() {
        return f;
    }

    /**
     * Sets the Feed-Forward coefficient of the PID controller gain.
     *
     * @param f Feed-Forward coefficient
     */
    public void setF(double f) {
        this.f = f;
    }

    /**
     * Returns the next output of the PID controller using Feed-Forward.
     *
     * @param measurement The current measurement of the process variable.
     */
    public double calculateWithKF(double measurement) {
        return calculate(measurement) + f * getSetpoint();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}
