package frc.robot.utilities;

/**
 * This class is used to store settings for different PIDs
 */
public class PIDCoefs {

  private double KP;
  private double KI;
  private double KD;
  private double KF;
  private double tolerance;
  private double deltaTolerance;

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
  public PIDCoefs(double KP, double KI, double KD, double tolerance, double deltaTolerance) {
    this(KP, KI, KD, 0, tolerance, deltaTolerance);
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
  public PIDCoefs(double KP, double KI, double KD, double KF, double tolerance, double deltaTolerance) {
    this.KP = KP;
    this.KI = KI;
    this.KD = KD;
    this.KF = KF;
    this.tolerance = tolerance;
    this.deltaTolerance = deltaTolerance;

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
}