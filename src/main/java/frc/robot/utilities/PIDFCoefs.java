package frc.robot.utilities;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

/**
 * This class is used to store settings for different PIDs
 */
public class PIDFCoefs implements Sendable {
    private double KP;
    private double KI;
    private double KD;
    private double KF;
    private double tolerance;
    private double deltaTolerance;
    private Constraints constraints;

    /**
     * @param KP             The Proportional coefficient of the PID loop in this
     *                       command.
     * @param KI             The Integral coefficient of the PID loop in this
     *                       command.
     * @param KD             The Differential coefficient of the PID loop in this
     *                       command.
     * @param KF             The Feed-Forward coefficient of the PID loop in this
     *                       command.
     * @param tolerance      The error tolerance of this command.
     * @param deltaTolerance The tolerance of the change in error.
     * @param constraints    constraints for a TrapezoidProfile
     */
    public PIDFCoefs(double KP, double KI, double KD, double KF, double tolerance, double deltaTolerance,
                     Constraints constraints) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = KF;
        this.tolerance = tolerance;
        this.deltaTolerance = deltaTolerance;
        this.constraints = constraints;
    }

    /**
     * @param KP             The Proportional coefficient of the PID loop in this
     *                       command.
     * @param KI             The Integral coefficient of the PID loop in this
     *                       command.
     * @param KD             The Differential coefficient of the PID loop in this
     *                       command.
     * @param tolerance      The error tolerance of this command.
     * @param deltaTolerance The tolerance of the change in error.
     * @param constraints    constraints for a TrapezoidProfile
     */
    public PIDFCoefs(double KP, double KI, double KD, double tolerance, double deltaTolerance, Constraints constraints) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.tolerance = tolerance;
        this.deltaTolerance = deltaTolerance;
        this.constraints = constraints;
    }

    /**
     * @param KP             The Proportional coefficient of the PID loop in this
     *                       command.
     * @param KI             The Integral coefficient of the PID loop in this
     *                       command.
     * @param KD             The Differential coefficient of the PID loop in this
     *                       command.
     * @param KF             The Feed-Forward coefficient of the PID loop in this
     *                       command.
     * @param tolerance      The error tolerance of this command.
     * @param deltaTolerance The tolerance of the change in error.
     */
    public PIDFCoefs(double KP, double KI, double KD, double KF, double tolerance, double deltaTolerance) {
        this(KP, KI, KD, KF, tolerance, deltaTolerance, new Constraints(0, 0));
    }

    /**
     * @param KP             The Proportional coefficient of the PID loop in this
     *                       command.
     * @param KI             The Integral coefficient of the PID loop in this
     *                       command.
     * @param KD             The Differential coefficient of the PID loop in this
     *                       command.
     * @param tolerance      The error tolerance of this command.
     * @param deltaTolerance The tolerance of the change in error.
     */
    public PIDFCoefs(double KP, double KI, double KD, double tolerance, double deltaTolerance) {
        this(KP, KI, KD, 0, tolerance, deltaTolerance);
    }

    /**
     * @param KP The Proportional coefficient of the PID loop in this command.
     * @param KI The Integral coefficient of the PID loop in this command.
     * @param KD The Differential coefficient of the PID loop in this command.
     * @param KF The Feed-Forward coefficient of the PID loop in this command.
     */
    public PIDFCoefs(double KP, double KI, double KD, double KF) {
        this(KP, KI, KD, KF, 0, 0);
    }

    /**
     * @param KP The Proportional coefficient of the PID loop in this command.
     * @param KI The Integral coefficient of the PID loop in this command.
     * @param KD The Differential coefficient of the PID loop in this command.
     */
    public PIDFCoefs(double KP, double KI, double KD) {
        this(KP, KI, KD, 0, 0, 0);
    }

    public PIDFCoefs(double KP, Constraints constraints) {
        this(KP, 0, 0, 0, 0, 0, constraints);
    }

    public double getKP() {
        return KP;
    }

    public void setKP(double KP) {
        this.KP = KP;
    }

    public double getKI() {
        return KI;
    }

    public void setKI(double KI) {
        this.KI = KI;
    }

    public double getKD() {
        return KD;
    }

    public void setKD(double KD) {
        this.KD = KD;
    }

    public double getKF() {
        return KF;
    }

    public void setKF(double KF) {
        this.KF = KF;
    }

    public double getTolerance() {
        return tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getDeltaTolerance() {
        return deltaTolerance;
    }

    public void setDeltaTolerance(double deltaTolerance) {
        this.deltaTolerance = deltaTolerance;
    }

    public Constraints getConstraints() {
        return constraints;
    }

    public void setConstraints(Constraints constraints) {
        this.constraints = constraints;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("p", this::getKP, this::setKP);
        builder.addDoubleProperty("i", this::getKI, this::setKI);
        builder.addDoubleProperty("d", this::getKD, this::setKD);
        builder.addDoubleProperty("f", this::getKF, this::setKF);
        builder.addDoubleProperty("tolerance", this::getTolerance, this::setTolerance);
        builder.addDoubleProperty("delta tolerance", this::getDeltaTolerance, this::setDeltaTolerance);
    }
}