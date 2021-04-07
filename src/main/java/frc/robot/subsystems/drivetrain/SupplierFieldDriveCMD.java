package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class SupplierFieldDriveCMD extends CommandBase {
    private final DrivetrainSS drivetrain;
    private final Supplier<Double> x, y, rot;
    double deadBand = 0.03 / 7;

    /**
     * Drives a drivetrain based on the given suppliers, relative to the field
     * (using thr drivetrain gyro).
     *
     * @param drivetrain the drivetrain to drive
     * @param x          the supplier for the field x power, between -1 and 1
     * @param y          the supplier for the field y power, between -1 and 1
     * @param rot        the supplier for the rotation power, between -1 and 1
     */
    public SupplierFieldDriveCMD(DrivetrainSS drivetrain, Supplier<Double> x, Supplier<Double> y,
                                 Supplier<Double> rot) {
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.rot = rot;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (Math.abs(x.get()) > deadBand ||
                Math.abs(y.get()) > deadBand ||
                Math.abs(rot.get()) > deadBand)
            drivetrain.fieldPowerDrive(x.get(), y.get(), rot.get());
        else drivetrain.stopDrive();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
    }
}
