package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class SupplierFieldDriveCMD extends CommandBase {
    private final DrivetrainSS drivetrain;
    Supplier<Double> x, y, rot;

    /**
     * Drives a drivetrain based on the given suppliers, relative to the field (using thr drivetrain gyro).
     * @param drivetrain the drivetrain to drive
     * @param x the supplier for the x velocity in m/s
     * @param y the supplier for the y velocity in m/s
     * @param rot the supplier for the rotation velocity in rad/s
     */
    public SupplierFieldDriveCMD(DrivetrainSS drivetrain, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot) {
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.rot = rot;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.fieldPowerDrive(x.get(), y.get(), rot.get());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
    }
}
