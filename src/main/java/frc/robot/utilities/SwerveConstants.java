package frc.robot.utilities;

public class SwerveConstants {

    public int speedID, angleID;
    public double diameter, offset;
    public PIDCoefs angleCoefs, speedCoefs;

    public SwerveConstants(int speedID, int angleID, double diameter, double offset, PIDCoefs angleCoefs, PIDCoefs speedCoefs) {
        this.speedID = speedID;
        this.angleID = angleID;
        this.diameter = diameter;
        this.offset = offset;
        this.angleCoefs = angleCoefs;
        this.speedCoefs = speedCoefs;
    }

}
