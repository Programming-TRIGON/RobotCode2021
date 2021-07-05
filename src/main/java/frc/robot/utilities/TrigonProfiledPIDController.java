package frc.robot.utilities;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class TrigonProfiledPIDController extends ProfiledPIDController {
    private double f;
    private boolean isTuning;

    public TrigonProfiledPIDController(PIDFCoefs pidCoefs) {
        super(pidCoefs.getKP(), pidCoefs.getKI(), pidCoefs.getKD(), pidCoefs.getConstraints());
        System.out.println(pidCoefs.getConstraints().maxAcceleration + " " + pidCoefs.getConstraints().maxVelocity);
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
        return calculate(measurement) + f * getSetpoint().position;
    }

    public boolean isTuning() {
        return isTuning;
    }

    public void setIsTuning(boolean isTuning) {
        this.isTuning = isTuning;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        // sends the pid values to the dashboard but only allows them to be changed if
        // isTuning is true
        builder.addDoubleProperty("p", this::getP, kP -> setP(isTuning ? kP : getP()));
        builder.addDoubleProperty("i", this::getI, kI -> setI(isTuning ? kI : getI()));
        builder.addDoubleProperty("d", this::getD, kD -> setD(isTuning ? kD : getD()));
        builder.addDoubleProperty("f", this::getF, kF -> setF(isTuning ? kF : getF()));
        builder.addDoubleProperty("setpoint", () -> getGoal().position,
                setpoint -> setGoal(isTuning ? setpoint : getSetpoint().position));
        builder.addBooleanProperty("isTuning", this::isTuning, this::setIsTuning);
    }
}
