package frc.robot.constants;

public class PIDWaypoint {
    public double xSetpoint;
    public boolean requireXSetpoint;
    public double ySetpoint;
    public boolean requireYSetpoint;
    public double zSetpoint;
    public boolean requireZSetpoint;

    public PIDWaypoint(double xSetpoint, boolean requireXSetpoint, double ySetpoint, boolean requireYSetpoint,
            double zSetpoint, boolean requireZSetpoint) {
        this.xSetpoint = xSetpoint;
        this.requireXSetpoint = requireXSetpoint;
        this.ySetpoint = ySetpoint;
        this.requireYSetpoint = requireYSetpoint;
        this.zSetpoint = zSetpoint;
        this.requireZSetpoint = requireZSetpoint;
    }
}