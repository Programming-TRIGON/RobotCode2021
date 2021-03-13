package frc.robot.utilities;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * This class is used to store settings for different PIDs
 */
public class SVACoefs implements Sendable {
    private double KS;
    private double KV;
    private double KA;

    /**
     * @param KS The static gain.
     * @param KV The velocity gain.
     * @param KA The acceleration gain.
     */
    public SVACoefs(double KS, double KV, double KA) {
        this.KS = KS;
        this.KV = KV;
        this.KA = KA;
    }

    public double getKS() {
        return KS;
    }

    public void setKS(double KS) {
        this.KS = KS;
    }

    public double getKV() {
        return KV;
    }

    public void setKV(double KV) {
        this.KV = KV;
    }

    public double getKA() {
        return KA;
    }

    public void setKA(double KA) {
        this.KA = KA;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("s", this::getKS, this::setKS);
        builder.addDoubleProperty("v", this::getKV, this::setKV);
        builder.addDoubleProperty("a", this::getKA, this::setKA);
    }
}
