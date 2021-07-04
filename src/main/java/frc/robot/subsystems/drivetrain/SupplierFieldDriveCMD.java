package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.DrivetrainConstants;
import frc.robot.utilities.TrigonPIDController;

import java.util.function.Supplier;

public class SupplierFieldDriveCMD extends CommandBase {
    private final DrivetrainSS drivetrain;
    private final Supplier<Double> x, y, rot;
    private final DrivetrainConstants drivetrainConstants;
    private final TrigonPIDController rotPID;
    double deadBand = 0.02;
    private double timeWhenRotDeadBand;
    private double timeToSetRotSetpoint;

    /**
     * Drives a drivetrain based on the given suppliers, relative to the field
     * (using thr drivetrain gyro).
     *
     * @param drivetrain the drivetrain to drive
     * @param x          the supplier for the field x power, between -1 and 1
     * @param y          the supplier for the field y power, between -1 and 1
     * @param rot        the supplier for the rotation power, between -1 and 1
     */
    public SupplierFieldDriveCMD(DrivetrainSS drivetrain, DrivetrainConstants drivetrainConstants, Supplier<Double> x,
            Supplier<Double> y, Supplier<Double> rot) {
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.drivetrainConstants = drivetrainConstants;
        this.rotPID = new TrigonPIDController(drivetrainConstants.ROTATION_PIDF_COEFS);
        rotPID.enableContinuousInput(-180, 180);
        timeToSetRotSetpoint = 1;
        SmartDashboard.putData("Drivetrain/RotPID", rotPID);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotPID.setSetpoint(drivetrain.getAngle());
        rotPID.reset();
        timeWhenRotDeadBand = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (Math.abs(rot.get()) > deadBand)
            timeWhenRotDeadBand = Timer.getFPGATimestamp();
        if (Timer.getFPGATimestamp() - timeWhenRotDeadBand < timeToSetRotSetpoint) {
            rotPID.setSetpoint(drivetrain.getAngle());
            rotPID.reset();
        }
        if (Math.abs(x.get()) > deadBand || Math.abs(y.get()) > deadBand || Math.abs(rot.get()) > deadBand)
            // !!!! THESE VALUES MIGHT LOOK DUMB BUT THIS IS THE ONLY WAY IT WORKS DO NOT
            // CHANGE !!!
            drivetrain.fieldPowerDrive(-x.get(), -y.get(), rot.get());
        else if (!rotPID.atSetpoint() && Timer.getFPGATimestamp() - timeWhenRotDeadBand > timeToSetRotSetpoint)
            drivetrain.fieldPowerDrive(0, 0, rotPID.calculate(drivetrain.getAngle()));
        else
            drivetrain.stopDrive();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
    }
}
