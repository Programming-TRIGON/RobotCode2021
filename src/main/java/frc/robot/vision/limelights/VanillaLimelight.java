package frc.robot.vision.limelights;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.constants.RobotConstants.LimelightConstants;
import frc.robot.vision.CamMode;
import frc.robot.vision.LedMode;
import frc.robot.vision.Target;

public class VanillaLimelight {

    protected final NetworkTableEntry tv, tx, ty, ta, ts, ledMode, camMode, pipeline, snapshot;
    protected final LimelightConstants constants;

    /**
     * @param tableKey the key of the limelight - if it was changed.
     */
    public VanillaLimelight(String tableKey, LimelightConstants constants) {
        this.constants = constants;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable limelightTable = inst.getTable(tableKey);
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        ts = limelightTable.getEntry("ts");
        ledMode = limelightTable.getEntry("ledMode");
        camMode = limelightTable.getEntry("camMode");
        pipeline = limelightTable.getEntry("pipeline");
        snapshot = limelightTable.getEntry("snapshot");
    }

    public VanillaLimelight(LimelightConstants constants) {
        this(constants.DEFAULT_TABLE_KEY, constants);
    }

    /**
     * @return the limelight constants
     */
    public LimelightConstants getLimelightConstants() {
        return constants;
    }

    /**
     * @return Whether the limelight has any valid targets (0 or 1)
     */
    public boolean hasTarget() {
        return tv.getDouble(0) != 0;
    }

    /**
     * @return Horizontal Offset From Crosshair To Target
     */
    public double getTx() {
        return tx.getDouble(0);
    }

    /**
     * @return Vertical Offset From Crosshair To Target
     */
    public double getTy() {
        return ty.getDouble(0);
    }

    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public double getTa() {
        return ta.getDouble(0);
    }

    /**
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getTs() {
        return ts.getDouble(0);
    }

    /**
     * @return The distance between the target and the middle of the robot
     */
//    public double getRobotTargetDistance() {
//        return calculateRobotToTargetVector().magnitude();
//    }

    /**
     * @return the angle from the middle of the robot to the target
     */
//    public double getAngle() {
//        Vector2d vector = calculateRobotToTargetVector();
//        return Math.toDegrees(Math.atan(vector.y / vector.x));
//    }

    /**
     * @return the cam mode in the NetworkTable.
     */
    public CamMode getCamMode() {
        return camMode.getDouble(0) == 0 ? CamMode.vision : CamMode.driver;
    }

    /**
     * @param camMode the mode to be changed to.
     */
    public void setCamMode(int camMode) {
        this.camMode.setNumber(camMode);
        NetworkTableInstance.getDefault().flush();
    }

    /**
     * @param camMode the mode to be changed to.
     */
    public void setCamMode(CamMode camMode) {
        setCamMode(camMode.getValue());
    }

    public void toggleLedMode() {
        if (getLedMode().equals(LedMode.off))
            setLedMode(LedMode.on);
        else
            setLedMode(LedMode.off);
    }

    /**
     * @return the led mode in the NetworkTable.
     */
    public LedMode getLedMode() {
        int index = (int) ledMode.getDouble(0);
        return LedMode.values()[index];
    }

    /**
     * @param ledMode the mode to be changed to.
     */
    public void setLedMode(LedMode ledMode) {
        setLedMode(ledMode.getValue());
    }

    /**
     * @param ledMode the mode to be changed to.
     */
    public void setLedMode(int ledMode) {
        this.ledMode.setNumber(ledMode);
        NetworkTableInstance.getDefault().flush();
    }

    /**
     * @return the current target in the NetworkTable.
     */
    public int getPipeline() {
        return (int) pipeline.getDouble(0);
    }

    /**
     * @param pipeline pipeline index to be changed to.
     */
    public void setPipeline(int pipeline) {
        this.pipeline.setNumber(pipeline);
        NetworkTableInstance.getDefault().flush();
    }

    /**
     * @param target the target to be changed to.
     */
    public void setPipeline(Target target) {
        setPipeline(target.getIndex());
    }

    /**
     * @param isTakingSnapshots If set to true, the limelight will start taking
     *                          snapshots. this is good for calibrating the
     *                          limelight when the target isn't available, for
     *                          example during a competition. If set to false, stops
     *                          taking snapshots.
     */
    public void setSnapshotState(boolean isTakingSnapshots) {
        snapshot.setNumber(isTakingSnapshots ? 1 : 0);
    }

    public void startVision(Target target) {
        setPipeline(target);
        setCamMode(CamMode.vision);
        setLedMode(LedMode.on);
    }

    /**
     * Stops the vision calculation, turns off led and change the camera to driver
     * mode.
     */
    public void stopVision() {
        //setCamMode(CamMode.driver);
        //setLedMode(LedMode.off);
    }

    /**
     * @return the vector between the middle of the robot and the target.
     */
//    private Vector2d calculateRobotToTargetVector() {
//        // This is the vector from the limelight to the target.
//        Vector2d limelightToTarget = new Vector2d(getTargetDistance(), 0);
//        limelightToTarget.rotate(getTx() + constants.LIMELIGHT_ANGLE_OFFSET);
//        // The offset is subtracted from the limelightToTarget vector in order to get
//        // the final vector.
//        return new Vector2d(limelightToTarget.x - constants.LIMELIGHT_OFFSET_X,
//                limelightToTarget.y - constants.LIMELIGHT_OFFSET_Y);
//    }

}
