package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class TrigonPIDController extends PIDController {
    private double f;
    private boolean isTuning;

    public TrigonPIDController(PIDCoefs pidCoefs) {
        super(pidCoefs.getKP(), pidCoefs.getKI(), pidCoefs.getKD());
        setTolerance(pidCoefs.getTolerance(), pidCoefs.getDeltaTolerance());
        f = pidCoefs.getKF();
        isTuning = false;
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

    public Object getIsTuning() {
        return this.isTuning;
    }

    public void setIsTuning(boolean isTuning) {
        this.isTuning = isTuning;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        // sends the pid values to the dashboard but only allows them to be changed if
        // isTuning is true
        builder.addDoubleProperty("p", this::getP, (kP) -> setP(isTuning ? kP : getP()));
        builder.addDoubleProperty("i", this::getI, (kI) -> setP(isTuning ? kI : getI()));
        builder.addDoubleProperty("d", this::getD, (kD) -> setP(isTuning ? kD : getD()));
        builder.addDoubleProperty("setpoint", this::getSetpoint,
                (setpoint) -> setSetpoint(isTuning ? setpoint : getSetpoint()));
        builder.addDoubleProperty("f", this::getF, (kF) -> setF(isTuning ? kF : getF()));
    }
}
